"""PX4: packed custom modes, AUTO.TAKEOFF then arm, OFFBOARD for sticks.

Two things PX4 does differently. A mode change is always accepted while
disarmed, so an ack proves nothing: every verb is confirmed on the
heartbeat or the landed state instead. And the takeoff order is mode
first, arm second.
"""

import logging
import time

from pymavlink import mavutil

from .link import BODY_NED
from .vehicle import Vehicle

logger = logging.getLogger(__name__)

NAN = float("nan")   # PX4 wants unused command params as NaN, never junk

MAIN = {"MANUAL": 1, "ALTCTL": 2, "POSCTL": 3, "AUTO": 4,
        "ACRO": 5, "OFFBOARD": 6, "STABILIZED": 7}
AUTO_SUB = {"READY": 1, "TAKEOFF": 2, "LOITER": 3, "MISSION": 4, "RTL": 5, "LAND": 6}

AUTO = MAIN["AUTO"]
TAKEOFF_CONFIRM_S = 20.0
TAKEOFF_TOLERANCE_M = 0.5   # PX4 settles a little under MIS_TAKEOFF_ALT
GCS_HEARTBEAT_S = 1.0


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


class PX4(Vehicle):

    name = "px4"
    # PX4's receiver takes only LOCAL_NED and BODY_NED; a body-offset frame is
    # dropped with "coordinate frame 9 unsupported" and OFFBOARD never engages
    velocity_frame = BODY_NED

    def __init__(self, link):
        super().__init__(link)
        self._heartbeat_at = 0.0

    def tick(self):
        # PX4 sends STATUSTEXT only to a link that has heartbeated as a GCS in
        # the last 2.5 s, so without this we never learn why anything failed.
        # ArduPilot must not get it: there a GCS heartbeat arms its GCS failsafe.
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
