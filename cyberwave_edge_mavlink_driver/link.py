"""The MAVLink side: one socket, one reader, a state cache.

One rule governs this file: exactly ONE thread (the driver's pump) calls
recv. Everyone else only sends, and waits on the state the pump keeps
fresh, never on the socket itself.
"""

import collections
import logging
import math
import threading
import time

from pymavlink import mavutil

logger = logging.getLogger(__name__)

VEL_MASK = 0b0000011111000111  # velocity + yaw rate, everything else ignored
HEARTBEAT_TIMEOUT_S = 3.0
STATUSTEXT_CHUNK = 50   # bytes per STATUSTEXT; longer lines arrive in pieces

# setpoint frames: ArduPilot takes body-offset, PX4 only takes these two
BODY_OFFSET_NED = mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED
BODY_NED = mavutil.mavlink.MAV_FRAME_BODY_NED

# HEARTBEAT types that are never the aircraft
NON_VEHICLE_HEARTBEAT_TYPES = frozenset({
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_TYPE_GIMBAL,
    mavutil.mavlink.MAV_TYPE_ADSB,
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
})

# the messages we read, requested at 10 Hz
STREAMED = (
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
)

LANDED_STATES = {
    mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND: "on_ground",
    mavutil.mavlink.MAV_LANDED_STATE_IN_AIR: "in_air",
    mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF: "in_air",
    mavutil.mavlink.MAV_LANDED_STATE_LANDING: "in_air",
}


def is_vehicle_heartbeat(msg):
    """True only for a HEARTBEAT that can be the autopilot's own."""
    if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
        return False
    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        return False
    if msg.type in NON_VEHICLE_HEARTBEAT_TYPES:
        return False
    return True


def enu_quaternion_from_ned_euler(roll, pitch, yaw):
    """ATTITUDE (NED world, FRD body) -> the twin's ENU quaternion.

    In: roll + = right wing down, pitch + = nose up, yaw = compass.
    Out: ENU world (x=East, y=North, z=Up) with an FLU body, the twin's
    URDF convention. The remap is the usual NED->ENU one (roll, -pitch,
    pi/2 - yaw); the quaternion is intrinsic Z-Y-X, keyed by name so the
    wire order cannot be misread.
    """
    r, p, y = roll, -pitch, math.pi / 2 - yaw
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return {
        "w": cr * cp * cy + sr * sp * sy,
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
    }


class MavlinkLink:
    """Connection plus the freshest state the autopilot has reported."""

    def __init__(self, connection):
        self.connection_string = connection
        self.m = None
        self.autopilot = None   # MAV_AUTOPILOT from the accepted heartbeat
        # written ONLY by pump_once(), read by everyone
        self.state = {
            "armed": False, "mode": None, "alt": 0.0, "ned": None,
            "attitude": None, "last_attitude": 0.0, "landed": None,
            "servo_pwm": None, "acks": {}, "last_heartbeat": 0.0,
        }
        # recent STATUSTEXT, so a verb can report why the autopilot refused
        self._texts = collections.deque(maxlen=64)
        self._texts_lock = threading.Lock()
        self._foreign_heartbeats = set()   # logged once each
        self._partial = None               # STATUSTEXT chunks still arriving

    # -- connection ----------------------------------------------------

    def connect(self, timeout=60.0):
        logger.info("connecting to %s", self.connection_string)
        self.m = mavutil.mavlink_connection(self.connection_string)
        # Behind mavlink-router the first heartbeat can carry sysid 0;
        # accepting it would turn every command into a broadcast.
        deadline = time.time() + timeout
        while time.time() < deadline:
            hb = self.m.wait_heartbeat(timeout=5)
            if hb is not None and is_vehicle_heartbeat(hb) and hb.get_srcSystem() != 0:
                self.m.target_system = hb.get_srcSystem()
                self.m.target_component = hb.get_srcComponent()
                self.autopilot = hb.autopilot
                break
        if self.autopilot is None:
            raise ConnectionError(f"no usable heartbeat on {self.connection_string}")
        logger.info("heartbeat: sys=%s comp=%s autopilot=%s",
                    self.m.target_system, self.m.target_component, self.autopilot)
        self.request_streams()

    def request_streams(self):
        """Ask for what we read at 10 Hz, and the landed state at 2 Hz."""
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        # PX4 ignores the legacy request above and streams slowly by default
        for msg_id in STREAMED:
            self.send_command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 100_000)
        self.send_command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                          mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 500_000)

    def connected(self):
        """A vehicle heartbeat within the last few seconds."""
        return time.time() - self.state["last_heartbeat"] < HEARTBEAT_TIMEOUT_S

    def close(self):
        if self.m is not None:
            self.m.close()

    # -- pump (call from ONE thread only) ------------------------------

    def _accept_heartbeat(self, msg):
        sysid, compid = msg.get_srcSystem(), msg.get_srcComponent()
        if is_vehicle_heartbeat(msg) and sysid in (self.m.target_system, 0):
            return True
        key = (sysid, compid, msg.type)
        if key not in self._foreign_heartbeats:
            self._foreign_heartbeats.add(key)
            name = getattr(mavutil.mavlink.enums["MAV_TYPE"].get(msg.type),
                           "name", msg.type)
            logger.info("ignoring non-vehicle HEARTBEAT: sys=%s comp=%s %s",
                        sysid, compid, name)
        return False

    def pump_once(self, timeout=1.0):
        """Receive one message, fold it into state, return its type."""
        msg = self.m.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        k = msg.get_type()
        s = self.state
        now = time.time()
        if k == "HEARTBEAT":
            if not self._accept_heartbeat(msg):
                return k
            s["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            s["mode"] = msg.custom_mode
            s["last_heartbeat"] = now
        elif k == "COMMAND_ACK":
            s["acks"][msg.command] = msg.result
        elif k == "GLOBAL_POSITION_INT":
            s["alt"] = msg.relative_alt / 1000.0
        elif k == "LOCAL_POSITION_NED":
            s["ned"] = (msg.x, msg.y, msg.z)
        elif k == "ATTITUDE":
            s["attitude"] = (msg.roll, msg.pitch, msg.yaw)
            s["last_attitude"] = now
        elif k == "SERVO_OUTPUT_RAW":
            s["servo_pwm"] = (msg.servo1_raw, msg.servo2_raw,
                              msg.servo3_raw, msg.servo4_raw)
        elif k == "EXTENDED_SYS_STATE":
            s["landed"] = LANDED_STATES.get(msg.landed_state)
        elif k == "STATUSTEXT":
            self._statustext(now, msg)
        return k

    def _statustext(self, now, msg):
        """Fold one STATUSTEXT in, joining the chunks of a long line.

        MAVLink 2 splits anything past 50 bytes into chunks that share an id
        and count up in chunk_seq. Without this, PX4's own words arrive cut:
        "Arming denied: Resolve system health failures firs" then "t".
        """
        text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
        if getattr(msg, "chunk_seq", 0) == 0:
            self._flush_text()
            self._partial = [now, text]
        elif self._partial is not None:
            self._partial[1] += text
        else:
            return                      # a tail whose head we never saw
        if len(text) < STATUSTEXT_CHUNK:
            self._flush_text()

    def _flush_text(self):
        if self._partial is None:
            return
        when, text = self._partial
        self._partial = None
        text = text.rstrip()    # PX4 ends its own log lines with a tab
        with self._texts_lock:
            self._texts.append((when, text))
        logger.info("[fc] %s", text)

    def texts_since(self, t0):
        """STATUSTEXT lines received at or after t0, oldest first."""
        with self._texts_lock:
            return [text for ts, text in self._texts if ts >= t0]

    # -- conversions ---------------------------------------------------

    def position_enu(self):
        """NED -> the twin's Z-up frame (x=east, y=north, z=up, never below 0)."""
        ned = self.state["ned"]
        if ned is None:
            return None
        return (ned[1], ned[0], max(0.0, -ned[2]))

    def attitude_quat_enu(self):
        """Latest ATTITUDE as the ENU quaternion the twin is told."""
        att = self.state["attitude"]
        if att is None:
            return None
        return enu_quaternion_from_ned_euler(*att)

    # -- sending (safe from any thread; never recv) --------------------

    def send_command(self, cmd, *params, fill=0.0):
        """COMMAND_LONG with seven params, the unused ones set to fill."""
        params = list(params) + [fill] * (7 - len(params))
        # only an ack that arrives after this send may explain the result
        self.state["acks"].pop(cmd, None)
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component, cmd, 0, *params)

    def wait_ack(self, cmd, timeout=5.0):
        """COMMAND_ACK result for cmd, or None if it never came."""
        end = time.time() + timeout
        while time.time() < end:
            if cmd in self.state["acks"]:
                return self.state["acks"].pop(cmd)
            time.sleep(0.05)
        return None

    def send_velocity_body(self, vx, vy, vz, yaw_rate, frame=BODY_OFFSET_NED):
        """One body-frame velocity setpoint. Callers stream this at 10 Hz."""
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component, frame,
            VEL_MASK, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate)
