"""MAVLink hardware layer — a headless GCS wrapping pymavlink.

One rule governs this file: exactly ONE thread (the driver's pump) calls
recv; every other thread only sends. Command methods therefore wait on
shared state that the pump keeps fresh, never on the socket itself.

Lessons baked in from the SITL spike (see ARCHITECTURE-NOTES.md session 3):
  - mode changes are verified via HEARTBEAT.custom_mode and retried
  - commands are trusted only on COMMAND_ACK
  - raw serial0 streams nothing until REQUEST_DATA_STREAM
  - a shared link carries heartbeats that are not the vehicle's
"""

import collections
import logging
import math
import threading
import time
from typing import Any, Optional

from pymavlink import mavutil

logger = logging.getLogger(__name__)

VEL_MASK = 0b0000011111000111  # velocity + yaw-rate control

# MAV_CMD_COMPONENT_ARM_DISARM param2: arm/disarm anyway, skip the checks
FORCE_ARM_MAGIC = 2989
FORCE_DISARM_MAGIC = 21196

# HEARTBEAT types that are never the aircraft
NON_VEHICLE_HEARTBEAT_TYPES = frozenset({
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_TYPE_GIMBAL,
    mavutil.mavlink.MAV_TYPE_ADSB,
    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
})


def is_vehicle_heartbeat(msg: Any) -> bool:
    """True only for a HEARTBEAT that can be the autopilot's own."""
    if msg.get_srcComponent() != mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
        return False
    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
        return False
    if msg.type in NON_VEHICLE_HEARTBEAT_TYPES:
        return False
    return True


def enu_quaternion_from_ned_euler(roll: float, pitch: float, yaw: float) -> dict:
    """ArduPilot ATTITUDE (NED world, FRD body) -> Cyberwave ENU quaternion.

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


class MavlinkVehicle:
    """Connection + state cache + command surface for one autopilot."""

    def __init__(self, connection: str) -> None:
        self.connection_string = connection
        self.m: Optional[Any] = None
        # written ONLY by pump_once(), read by everyone
        self.state: dict[str, Any] = {
            "armed": False, "mode": None, "mode_name": None, "alt": 0.0,
            "ned": None, "attitude": None, "acks": {}, "last_heartbeat": 0.0,
            "servo_pwm": None,
        }
        # recent STATUSTEXT, so a command can report why the FC refused it
        self._texts: "collections.deque[tuple]" = collections.deque(maxlen=64)
        self._texts_lock = threading.Lock()
        self._foreign_heartbeats: set = set()   # logged once each

    # -- connection ----------------------------------------------------

    def connect(self, timeout: float = 60.0) -> None:
        logger.info("MAVLink connecting to %s", self.connection_string)
        self.m = mavutil.mavlink_connection(self.connection_string)
        # Behind mavlink-router the first heartbeat can carry sysid 0
        # (seen on the Pi, 2026-08-30); accepting it would turn every
        # command into a broadcast. Wait for a real system id.
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.m.wait_heartbeat(timeout=5) is not None \
                    and self.m.target_system != 0:
                break
        if self.m.target_system == 0:
            raise ConnectionError(f"no usable heartbeat on {self.connection_string}")
        logger.info("heartbeat: sys=%s comp=%s", self.m.target_system, self.m.target_component)
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # -- pump (call from ONE thread only) ------------------------------

    def _accept_heartbeat(self, msg: Any) -> bool:
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

    def pump_once(self, timeout: float = 1.0) -> Optional[str]:
        """Receive one message, fold it into state, return its type."""
        msg = self.m.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        k = msg.get_type()
        s = self.state
        if k == "HEARTBEAT":
            if not self._accept_heartbeat(msg):
                return k
            s["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            s["mode"] = msg.custom_mode
            s["mode_name"] = mavutil.mode_string_v10(msg)
            s["last_heartbeat"] = time.time()
        elif k == "COMMAND_ACK":
            s["acks"][msg.command] = msg.result
        elif k == "GLOBAL_POSITION_INT":
            s["alt"] = msg.relative_alt / 1000.0
        elif k == "LOCAL_POSITION_NED":
            s["ned"] = (msg.x, msg.y, msg.z)
        elif k == "ATTITUDE":
            s["attitude"] = (msg.roll, msg.pitch, msg.yaw)
        elif k == "SERVO_OUTPUT_RAW":
            s["servo_pwm"] = (msg.servo1_raw, msg.servo2_raw,
                              msg.servo3_raw, msg.servo4_raw)
        elif k == "STATUSTEXT":
            text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
            with self._texts_lock:
                self._texts.append((time.time(), text))
            logger.info("[fc] %s", text)
        return k

    # -- conversions ---------------------------------------------------

    def position_enu(self) -> Optional[tuple]:
        """NED -> Cyberwave Z-up (x=east, y=north, z=up, clamped >= 0)."""
        ned = self.state["ned"]
        if ned is None:
            return None
        return (ned[1], ned[0], max(0.0, -ned[2]))

    def attitude_quat_enu(self) -> Optional[dict]:
        """Latest ATTITUDE as the ENU quaternion the twin is told."""
        att = self.state["attitude"]
        if att is None:
            return None
        return enu_quaternion_from_ned_euler(*att)

    # -- commands (safe from any thread; never recv) -------------------

    def _wait_ack(self, cmd: int, timeout: float = 5.0) -> Optional[int]:
        end = time.time() + timeout
        while time.time() < end:
            if cmd in self.state["acks"]:
                return self.state["acks"].pop(cmd)
            time.sleep(0.05)
        return None

    def ensure_mode(self, name: str, timeout: float = 30.0) -> bool:
        want = self.m.mode_mapping()[name]
        end = time.time() + timeout
        while time.time() < end:
            self.m.set_mode(name)
            time.sleep(1.0)
            if self.state["mode"] == want:
                logger.info("mode %s confirmed", name)
                return True
        logger.error("could not enter mode %s", name)
        return False

    def set_armed(self, arm: bool, force: bool = False,
                  timeout: float = 5.0) -> tuple:
        """Arm or disarm, then wait for the heartbeat to agree.

        Returns (ok, reason). On refusal the reason is the FC's own words
        from STATUSTEXT, or the COMMAND_ACK result if it said nothing.
        """
        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        p1 = 1 if arm else 0
        p2 = (FORCE_ARM_MAGIC if arm else FORCE_DISARM_MAGIC) if force else 0
        t0 = time.time()
        self.state["acks"].pop(cmd, None)
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            cmd, 0, p1, p2, 0, 0, 0, 0, 0)
        logger.info("%s sent (force=%s)", "arm" if arm else "disarm", force)

        end = t0 + timeout
        while time.time() < end:
            if self.state["armed"] == arm:
                return True, ""
            time.sleep(0.05)

        with self._texts_lock:
            texts = [t for ts, t in self._texts if ts >= t0]
        texts = [t for t in texts if t.lower().startswith(("arm", "prearm"))] or texts
        result = self.state["acks"].pop(cmd, None)
        if texts:
            reason = "; ".join(dict.fromkeys(texts))
        elif result is not None:
            reason = getattr(mavutil.mavlink.enums["MAV_RESULT"].get(result),
                             "name", f"MAV_RESULT {result}")
        else:
            reason = f"no armed-state change and no COMMAND_ACK within {timeout:.1f}s"
        logger.warning("%s refused: %s", "arm" if arm else "disarm", reason)
        return False, reason

    def arm(self, timeout: float = 120.0) -> bool:
        """Keep asking until the vehicle arms (the takeoff path)."""
        end = time.time() + timeout
        while time.time() < end and not self.state["armed"]:
            self.set_armed(True, timeout=3.0)
        if self.state["armed"]:
            logger.info("armed")
            return True
        logger.error("arming timed out (PreArm reasons in [fc] log lines)")
        return False

    def takeoff(self, altitude: float) -> bool:
        if not self.ensure_mode("GUIDED"):
            return False
        if not self.arm():
            return False
        for _ in range(5):
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
            if self._wait_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF) == \
                    mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("takeoff accepted -> %.1f m", altitude)
                return True
            time.sleep(2.0)
        logger.error("NAV_TAKEOFF never accepted")
        return False

    def land(self) -> bool:
        return self.ensure_mode("LAND")

    def return_to_home(self) -> bool:
        return self.ensure_mode("RTL")

    def emergency_disarm(self, timeout: float = 3.0) -> bool:
        """Force-disarm, confirmed by the heartbeat's armed bit dropping."""
        logger.warning("EMERGENCY force-disarm sent")
        ok, reason = self.set_armed(False, force=True, timeout=timeout)
        if ok:
            logger.info("disarm confirmed")
        else:
            logger.error("armed bit still set %.1fs after force-disarm: %s",
                         timeout, reason)
        return ok

    def send_velocity_body(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """One body-frame velocity setpoint. Callers stream this at ~10 Hz."""
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VEL_MASK, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate)

    def disconnect(self) -> None:
        if self.m is not None:
            self.m.close()
