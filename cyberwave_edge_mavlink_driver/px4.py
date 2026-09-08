"""PX4: packed custom modes, AUTO.TAKEOFF then arm, OFFBOARD for sticks.

Two things PX4 does differently. A mode change is always accepted while
disarmed, so an ack proves nothing: every verb is confirmed on the
heartbeat or the landed state instead. And the takeoff order is mode
first, arm second.
"""

import logging
import math
import time

from pymavlink import mavutil

from .link import BODY_NED
from .vehicle import Refused, Vehicle, result_name

logger = logging.getLogger(__name__)

NAN = float("nan")   # PX4 wants unused command params as NaN, never junk

MAIN = {"MANUAL": 1, "ALTCTL": 2, "POSCTL": 3, "AUTO": 4,
        "ACRO": 5, "OFFBOARD": 6, "STABILIZED": 7}
AUTO_SUB = {"READY": 1, "TAKEOFF": 2, "LOITER": 3, "MISSION": 4, "RTL": 5, "LAND": 6}

AUTO = MAIN["AUTO"]
TAKEOFF_CONFIRM_S = 20.0
TAKEOFF_TOLERANCE_M = 0.5   # PX4 settles a little under MIS_TAKEOFF_ALT
GCS_HEARTBEAT_S = 1.0

# GIMBAL_MANAGER_FLAGS, the gimbal v2 word for which frame an angle is in.
# Locked means held against the world; without the flags the angle is a
# body angle and follows the aircraft round.
PITCH_LOCK = 8
YAW_LOCK = 16
EARTH_FRAME = PITCH_LOCK | YAW_LOCK
ALL_GIMBALS = 0             # device id 0: whichever mount the manager has
GIMBAL_ACK_S = 2.0
CALIBRATION_S = 3.0         # the ack is quick; the [cal] line follows it
CALIBRATION_TRIES = 8       # a cancel lands between sampling steps, not during one
NO_GIMBAL = "not supported on this vehicle"


def custom_mode(main, sub=0):
    """HEARTBEAT.custom_mode for a PX4 main and sub mode."""
    return (sub << 24) | (main << 16)


def mode_name(custom):
    main, sub = (custom >> 16) & 0xFF, (custom >> 24) & 0xFF
    if main == AUTO:
        subs = {v: k for k, v in AUTO_SUB.items()}
        return "AUTO." + subs.get(sub, str(sub))
    mains = {v: k for k, v in MAIN.items()}
    return mains.get(main, f"MODE({main},{sub})")


def refusal(reason):
    """A refusal in the contract's words where they fit, else the FC's own.

    Silence and UNSUPPORTED say the same thing to a caller: this aircraft
    does not do that. Everything else is the autopilot's own verdict and
    goes back untouched.
    """
    if reason.startswith("no COMMAND_ACK") or reason == "MAV_RESULT_UNSUPPORTED":
        return NO_GIMBAL
    return reason


def pitch_yaw_degrees(q):
    """A gimbal attitude quaternion (w, x, y, z) as pitch and yaw in degrees.

    The mount builds it from an intrinsic Z-Y-X euler with no roll, so the
    two angles come straight back out; asin is clamped because a quaternion
    off the wire is only float-accurate.
    """
    w, x, y, z = q
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return round(math.degrees(pitch), 2), round(math.degrees(yaw), 2)


class PX4(Vehicle):

    name = "px4"
    # PX4's receiver takes only LOCAL_NED and BODY_NED; a body-offset frame is
    # dropped with "coordinate frame 9 unsupported" and OFFBOARD never engages
    velocity_frame = BODY_NED

    def __init__(self, link):
        super().__init__(link)
        self._heartbeat_at = 0.0
        self._gimbal = None     # None until we have asked the manager for control
        self._claim_at = None   # when the tick's own CONFIGURE went out

    def tick(self):
        # PX4 sends STATUSTEXT only to a link that has heartbeated as a GCS in
        # the last 2.5 s, so without this we never learn why anything failed.
        # ArduPilot must not get it: there a GCS heartbeat arms its GCS failsafe.
        if not self.link.ready():
            return          # the link is being rebuilt; nothing goes into the gap
        now = time.time()
        if now - self._heartbeat_at >= GCS_HEARTBEAT_S:
            self._heartbeat_at = now
            self.link.m.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def mode_name(self):
        mode = self.link.state["mode"]
        return mode_name(mode) if mode is not None else "MODE(None)"

    def returning(self):
        return self.link.state["mode"] == custom_mode(AUTO, AUTO_SUB["RTL"])

    def set_mode(self, main, sub=0, timeout=10.0, meanwhile=None):
        """DO_SET_MODE(main, sub), confirmed on the heartbeat."""
        want = custom_mode(main, sub)
        return self._set_mode(mode_name(want), want, (main, sub), timeout,
                              fill=NAN, meanwhile=meanwhile)

    def set_armed(self, arm, force=False, timeout=5.0):
        if arm and force:
            # Commander runs its checks for anything that arrives over MAVLink
            logger.warning("force arm is not possible here, arming with checks")
            force = False
        return super().set_armed(arm, force, timeout)

    def takeoff(self, altitude):
        # the takeoff altitude is a parameter, not a command argument
        if not self.link.ready():
            return False, "not connected"
        self.link.m.mav.param_set_send(
            self.link.m.target_system, self.link.m.target_component,
            b"MIS_TAKEOFF_ALT", float(altitude), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        ok, reason = self.set_mode(AUTO, AUTO_SUB["TAKEOFF"])
        if not ok:
            return ok, reason
        ok, reason = self.set_armed(True)
        if not ok:
            return ok, reason
        # PX4 calls the landed state in_air the moment the climb starts, so
        # that alone hands back an aircraft still on the ground: wait for the
        # altitude too, or the next verb runs at zero and nothing moves
        want = altitude - TAKEOFF_TOLERANCE_M
        if self._wait(lambda: self.in_air() and self.link.state["alt"] >= want,
                      TAKEOFF_CONFIRM_S):
            logger.info("airborne at %.1f m", self.link.state["alt"])
            return True, ""
        if self.in_air():
            return False, (f"still climbing, {self.link.state['alt']:.1f} m of "
                           f"{altitude:.1f} m after {TAKEOFF_CONFIRM_S:.0f}s")
        return False, "armed but never left the ground"

    def takeoff_altitude(self, asked):
        # takeoff here returns only once the altitude is reached, so the
        # reading at that moment is the height the aircraft really got to
        alt = self.link.state["alt"]
        return round(alt, 2) if self.in_air() and alt > 0 else asked

    def land(self):
        return self.set_mode(AUTO, AUTO_SUB["LAND"])

    def return_to_home(self):
        return self.set_mode(AUTO, AUTO_SUB["RTL"])

    def hold(self):
        return self.set_mode(AUTO, AUTO_SUB["LOITER"])

    def prepare_sticks(self):
        """OFFBOARD only engages with a setpoint stream already running."""
        if not self.armed() or self.link.state["mode"] == custom_mode(MAIN["OFFBOARD"]):
            return
        for _ in range(5):
            self.send_velocity_body(0, 0, 0, 0)
            time.sleep(0.1)
        self.set_mode(MAIN["OFFBOARD"], timeout=3.0)

    def release_sticks(self):
        """Leave OFFBOARD for Hold with the zeros still flowing.

        PX4 fails over a second after the setpoint stream stops, and on the
        bench that ended in an RTL nobody asked for; the stream must outlive
        the mode change.
        """
        def zero():
            self.send_velocity_body(0, 0, 0, 0)
        zero()
        if self.link.state["mode"] == custom_mode(MAIN["OFFBOARD"]):
            self.set_mode(AUTO, AUTO_SUB["LOITER"], timeout=3.0, meanwhile=zero)

    # -- gimbal, home point, compass ------------------------------------

    def gimbal_point(self, pitch_deg, yaw_deg, absolute, duration_s=None):
        """DO_GIMBAL_MANAGER_PITCHYAW, once the manager is ours.

        The command carries a target, never a delta, so a relative move is
        read back and added here and goes on the wire as the angle it comes
        to. The lock flags are not the place for it: they say which frame
        the angle is measured in, and sending a body angle instead left
        +15 after -45 sitting at +15 rather than -30.
        """
        if duration_s is not None:
            # DJI takes a rotation time; the gimbal manager has no such field
            # and moves at the mount's own speed, so there is nothing to send
            logger.info("px4 has no gimbal slew time, ignoring duration %.1fs", duration_s)
        if not absolute:
            here = self.gimbal_attitude()
            if here is None:
                raise Refused("no gimbal attitude to move from")
            # an axis left at NaN stays NaN through the addition, and so
            # stays the axis nobody commanded
            pitch_deg, yaw_deg = here[0] + pitch_deg, here[1] + yaw_deg
        self._take_gimbal()
        ok, reason = self._acked(
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            float(pitch_deg), float(yaw_deg), NAN, NAN,
            EARTH_FRAME, 0, ALL_GIMBALS, timeout=GIMBAL_ACK_S)
        if not ok:
            raise Refused(refusal(reason))
        logger.info("gimbal to pitch %.1f yaw %.1f", pitch_deg, yaw_deg)

    def gimbal_rate(self, pitch_dps, yaw_dps):
        """GIMBAL_MANAGER_SET_ATTITUDE with a rate and no angle.

        Not the command's own rate fields: PX4 reads a NaN angle there as
        zero, so every refresh drags the mount back to centre and a held
        stick moves it a tenth of a degree. The message leaves a NaN
        quaternion alone, and that is what lets the rate add up. Nothing
        acks it, and PX4 stops the mount itself 2 s after the last one.

        This runs on the tick, so the manager is claimed without waiting for
        the ack: two seconds here is two seconds with no flight setpoint, and
        PX4 gives up on OFFBOARD after one.
        """
        if self._gimbal is None:
            self._gimbal = self._claim_gimbal()
        if self._gimbal is None:
            return                  # the claim is out; its ack lands on a later pass
        if not self._gimbal:
            raise Refused(NO_GIMBAL)
        self.link.m.mav.gimbal_manager_set_attitude_send(
            self.link.m.target_system, self.link.m.target_component,
            0, ALL_GIMBALS, [NAN] * 4, NAN,
            math.radians(pitch_dps), math.radians(yaw_dps))

    def gimbal_attitude(self):
        """Where the mount is pointing now, (pitch, yaw) in degrees.

        The pump caches every message it reads, including the two the mount
        sends; reading that cache is safe from here because nothing but the
        pump ever calls recv. GIMBAL_DEVICE_ATTITUDE_STATUS is the gimbal v2
        answer and comes from the mount itself; MOUNT_ORIENTATION is the
        older one, and on a PX4 driving a servo it is the only one that shows
        the angle actually reached rather than the angle asked for.
        """
        if not self.link.ready():
            return None
        msg = self.link.m.messages.get("GIMBAL_DEVICE_ATTITUDE_STATUS")
        if msg is not None:
            return pitch_yaw_degrees(msg.q)
        msg = self.link.m.messages.get("MOUNT_ORIENTATION")
        if msg is not None:
            return round(msg.pitch, 2), round(msg.yaw, 2)
        return None

    def set_home(self, lat, lon, alt_m=None):
        """DO_SET_HOME with a point of our own, as COMMAND_INT.

        param1 = 0 means "take the coordinates in this message". They go as
        COMMAND_INT because a COMMAND_LONG carries the latitude in a float32
        and puts home a foot or two from where it was asked for. PX4 denies
        a non-finite altitude, so with none given we send the aircraft's own
        height above sea level.
        """
        if not self.link.ready():
            raise Refused("not connected")
        if alt_m is None:
            alt_m = self.amsl()
            if alt_m is None:
                raise Refused("no position fix")
        cmd = mavutil.mavlink.MAV_CMD_DO_SET_HOME
        acks = self.link.state["acks"]
        acks.pop(cmd, None)
        self.link.m.mav.command_int_send(
            self.link.m.target_system, self.link.m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL, cmd, 0, 0,
            0, 0, 0, NAN, int(round(lat * 1e7)), int(round(lon * 1e7)), float(alt_m))
        if not self._wait(lambda: cmd in acks, GIMBAL_ACK_S):
            raise Refused(f"no COMMAND_ACK within {GIMBAL_ACK_S:.1f}s")
        result = acks.pop(cmd)
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise Refused(result_name(result))
        logger.info("home set to %.7f %.7f %.1f m", lat, lon, alt_m)

    def compass_calibration(self, start):
        """PREFLIGHT_CALIBRATION: param2 = 1 starts the magnetometer.

        Every param zero is the cancel, and it is not the commander that
        reads it but the calibration itself, between its sampling steps.
        Sent in the middle of one it is answered TEMPORARILY_REJECTED, by
        the commander whose worker is busy, and nothing stops — so the
        cancel goes out again until PX4 prints "[cal] calibration
        cancelled", the same words that prove a start took.
        """
        if self.armed():
            raise Refused("motors running")
        cmd = mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
        word = "started" if start else "cancelled"
        t0 = time.time()

        def said_so():
            return any(word in t for t in self.link.texts_since(t0))

        reason = "nothing came back"
        for _ in range(1 if start else CALIBRATION_TRIES):
            ok, reason = self._acked(cmd, 0, 1 if start else 0, timeout=CALIBRATION_S)
            if ok or self._wait(said_so, CALIBRATION_S if start else 0.5):
                logger.info("mag calibration %s", word)
                return
        raise Refused(refusal(reason))

    def amsl(self):
        """Altitude above mean sea level, from the pump's last fix."""
        msg = self.link.m.messages.get("GLOBAL_POSITION_INT") if self.link.ready() else None
        return None if msg is None else msg.alt / 1000.0

    def _take_gimbal(self):
        """Become the gimbal manager's primary controller.

        PX4 denies every pitchyaw command from anyone else, and an aircraft
        with no gimbal module running never answers at all: the commander
        leaves both gimbal commands to a module that is not there. This one
        waits for the ack, so only a verb thread may call it.
        """
        ok, reason = self._acked(
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,
            self.link.m.mav.srcSystem, self.link.m.mav.srcComponent, -1, -1,
            0, 0, ALL_GIMBALS, timeout=GIMBAL_ACK_S)
        self._gimbal = ok
        if not ok:
            raise Refused(refusal(reason))

    def _claim_gimbal(self):
        """The same claim for the tick: send once, read the ack later.

        True once the manager is ours, False when it was refused or nothing
        answered in time, None while the ack is still out.
        """
        if not self.link.ready():
            return None
        cmd = mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
        if self._claim_at is None:
            self._claim_at = time.time()
            self.link.send_command(cmd, self.link.m.mav.srcSystem,
                                   self.link.m.mav.srcComponent, -1, -1,
                                   0, 0, ALL_GIMBALS)
        result = self.link.state["acks"].pop(cmd, None)
        if result is not None:
            return result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        return None if time.time() - self._claim_at < GIMBAL_ACK_S else False
