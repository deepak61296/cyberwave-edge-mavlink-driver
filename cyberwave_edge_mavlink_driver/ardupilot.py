"""ArduPilot: modes by name, GUIDED for takeoff and sticks, BRAKE to hold."""

import logging
import time

from pymavlink import mavutil

from .contract import NOT_SUPPORTED
from .link import GLOBAL_AMSL, GLOBAL_RELATIVE_ALT
from .vehicle import NAN, Refused, Vehicle, refusal

logger = logging.getLogger(__name__)

ARM_RETRY_S = 30.0      # takeoff keeps asking this long; pre-arm can take a while
ARM_TRY_S = 3.0         # each ask waits this long for the armed bit
AIRBORNE_S = 20.0       # how long the climb has to show after the ack
AIRBORNE_ALT_M = 0.5    # off the ground, when the landed state says nothing
MAX_SLEW_S = 30.0       # the longest a gimbal move may be asked to take

# MAV_CMD_DO_START_MAG_CAL: every compass, no retry, save it when it finishes.
# Saving is what makes the separate ACCEPT command unnecessary.
MAG_CAL_ALL = 0
MAG_CAL_NO_RETRY = 0
MAG_CAL_AUTOSAVE = 1


def _add(base, delta):
    """base + delta, leaving an axis nobody commanded uncommanded."""
    return NAN if delta != delta else base + delta


def _rate(here, there, seconds):
    """deg/s that covers the gap in the time asked for."""
    return NAN if there != there else (there - here) / seconds


def _both(a, b, spare):
    """ArduPilot refuses a pitch/yaw pair with one half NaN, so an axis
    nobody commanded is filled with what leaves that axis where it is."""
    return (spare[0] if a != a else a, spare[1] if b != b else b)


class ArduPilot(Vehicle):

    name = "ardupilot"

    def __init__(self, link):
        super().__init__(link)
        # the table for the system we accepted; pymavlink's own mode_mapping()
        # follows whichever heartbeat it saw first, which differs on a shared link
        kind = link.m.sysid_state[link.m.target_system].mav_type
        self.modes = mavutil.mode_mapping_byname(kind) or {}
        self._names = {v: k for k, v in self.modes.items()}

    def mode_name(self):
        mode = self.link.state["mode"]
        return self._names.get(mode, f"MODE({mode})")

    def returning(self):
        return self.mode_name() in ("RTL", "SMART_RTL")

    def set_mode(self, name, timeout=30.0):
        """Ask for a mode by name until the heartbeat shows it."""
        want = self.modes.get(name)
        if want is None:
            return False, f"unknown mode {name}"
        return self._set_mode(name, want, (want,), timeout)

    def set_armed(self, arm, force=False, timeout=5.0):
        # LAND is not armable, the bench found out
        if arm and self.mode_name() == "LAND":
            self.set_mode("GUIDED", timeout=5.0)
        return super().set_armed(arm, force, timeout)

    def takeoff(self, altitude):
        ok, reason = self.set_mode("GUIDED")
        if not ok:
            return ok, reason
        end = time.time() + ARM_RETRY_S
        while True:
            ok, reason = self.set_armed(True, timeout=ARM_TRY_S)
            if ok or self.abort.is_set() or time.time() > end:
                break
        if not ok:
            return False, reason
        cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        for _ in range(5):
            ok, reason = self._acked(cmd, 0, 0, 0, 0, 0, 0, altitude)
            if ok:
                logger.info("takeoff accepted, %.1f m", altitude)
                return self._airborne()
            if self.abort.wait(2.0):    # the pause between tries, unless cut short
                break
        return self.takeoff_failed("NAV_TAKEOFF not accepted")

    def _airborne(self):
        """Hold the reply until the aircraft is actually up, as PX4 does.

        The ack only says NAV_TAKEOFF was taken. Answering on it hands back
        an aircraft still on the ground, and the next verb runs at zero.
        """
        if self._wait(lambda: self.in_air() or self.link.state["alt"] > AIRBORNE_ALT_M,
                      AIRBORNE_S):
            logger.info("airborne at %.1f m", self.link.state["alt"])
            return True, ""
        return self.takeoff_failed("armed but never left the ground")

    def land(self):
        return self.set_mode("LAND")

    def return_to_home(self):
        return self.set_mode("RTL")

    def hold(self):
        # Armed is the test, not airborne: a takeoff cancelled in its first
        # second is still on the ground by the landed state, and NAV_TAKEOFF
        # goes on climbing under a hold that does nothing. Disarmed there is
        # nothing to stop, and the contract refuses the verb anyway.
        if not self.armed():
            return True, ""
        return self.set_mode("BRAKE")

    def prepare_sticks(self):
        # GUIDED is the mode that takes velocity setpoints
        if self.armed() and self.mode_name() != "GUIDED":
            self.set_mode("GUIDED", timeout=5.0)

    def release_sticks(self):
        self.send_velocity_body(0, 0, 0, 0)

    # -- camera ----------------------------------------------------------

    def gimbal_attitude(self):
        return self.link.state["gimbal"]

    def gimbal_point(self, pitch_deg, yaw_deg, absolute, duration_s=None):
        if not absolute:
            here = self.gimbal_attitude()
            if here is None:
                raise Refused("no gimbal attitude to move from")
            pitch_deg, yaw_deg = _add(here[0], pitch_deg), _add(here[1], yaw_deg)
        if duration_s and duration_s > 0:
            return self._slew(pitch_deg, yaw_deg, min(duration_s, MAX_SLEW_S))
        self._angles(pitch_deg, yaw_deg)

    def _slew(self, pitch_deg, yaw_deg, seconds):
        """A move that takes the time it was asked to take.

        The gimbal protocol has no such thing, so this drives the rate that
        covers the gap and then lands on the angle exactly.
        """
        here = self.gimbal_attitude()
        if here is None:
            return self._angles(pitch_deg, yaw_deg)
        self.gimbal_rate(_rate(here[0], pitch_deg, seconds),
                         _rate(here[1], yaw_deg, seconds))
        self.abort.wait(seconds)
        self._angles(pitch_deg, yaw_deg)

    def gimbal_rate(self, pitch_dps, yaw_dps):
        # streamed from the tick, so it sends and does not wait for the ack.
        # Nothing acks it either, so a mount that has never reported an angle
        # is the one sign there is none to turn, and PX4 says the same words.
        if self.gimbal_attitude() is None:
            raise Refused(NOT_SUPPORTED)
        pitch_dps, yaw_dps = _both(pitch_dps, yaw_dps, (0.0, 0.0))
        self.link.send_command(mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
                               NAN, NAN, pitch_dps, yaw_dps, 0, 0, 0)

    def _angles(self, pitch_deg, yaw_deg):
        """The gimbal protocol v2 verb, carrying the two angles.

        An axis the caller left out keeps the angle the gimbal already holds,
        because the firmware takes the pair or nothing.
        """
        pitch_deg, yaw_deg = _both(pitch_deg, yaw_deg,
                                   self.gimbal_attitude() or (0.0, 0.0))
        ok, reason = self._acked(
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            pitch_deg, yaw_deg, NAN, NAN, 0, 0, 0)
        if not ok:
            raise Refused(refusal(reason))

    # -- home and compass -------------------------------------------------

    def set_home(self, lat, lon, alt_m):
        # as COMMAND_INT, so the coordinates arrive whole; param1 0 means the
        # point is the one in x, y and z rather than where the aircraft is.
        # An altitude is AMSL, as every global altitude in the contract is;
        # with none given, zero in the home-relative frame is the height home
        # already has. RTL descends to this, so the frame is not a detail.
        ok, reason = self._acked_int(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0,
                                     x=int(round(lat * 1e7)),
                                     y=int(round(lon * 1e7)),
                                     z=0.0 if alt_m is None else float(alt_m),
                                     frame=(GLOBAL_RELATIVE_ALT if alt_m is None
                                            else GLOBAL_AMSL))
        if not ok:
            raise Refused(reason)

    def compass_calibration(self, start):
        if start:
            ok, reason = self._acked(mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
                                     MAG_CAL_ALL, MAG_CAL_NO_RETRY, MAG_CAL_AUTOSAVE)
        else:
            ok, reason = self._acked(mavutil.mavlink.MAV_CMD_DO_CANCEL_MAG_CAL,
                                     MAG_CAL_ALL)
        if not ok:
            raise Refused(reason)
