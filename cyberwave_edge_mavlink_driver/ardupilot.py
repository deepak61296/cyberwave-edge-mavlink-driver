"""ArduPilot: modes by name, GUIDED for takeoff and sticks, BRAKE to hold."""

import logging
import time

from pymavlink import mavutil

from .vehicle import Vehicle

logger = logging.getLogger(__name__)

ARM_RETRY_S = 30.0   # takeoff keeps asking this long; pre-arm can take a while


class ArduPilot(Vehicle):

    name = "ardupilot"

    def mode_name(self):
        by_number = {v: k for k, v in (self.link.m.mode_mapping() or {}).items()}
        mode = self.link.state["mode"]
        return by_number.get(mode, f"MODE({mode})")

    def returning(self):
        return self.mode_name() in ("RTL", "SMART_RTL")

    def set_mode(self, name, timeout=30.0):
        """Ask for a mode once a second until the heartbeat shows it."""
        want = (self.link.m.mode_mapping() or {}).get(name)
        if want is None:
            return False, f"unknown mode {name}"
        t0 = time.time()

        def in_mode():
            return self.link.state["mode"] == want

        while not in_mode():
            if self.abort.is_set() or time.time() > t0 + timeout:
                # the autopilot usually says why on the text channel; prefer its words
                why = "; ".join(dict.fromkeys(self.link.texts_since(t0)))
                logger.error("could not enter mode %s: %s", name, why)
                return False, why or f"could not enter {name} within {timeout:.0f}s"
            self.link.m.set_mode(name)
            self._wait(in_mode, min(1.0, t0 + timeout - time.time()))
        return True, ""

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
            ok, reason = self.set_armed(True, timeout=3.0)
            if ok or self.abort.is_set() or time.time() > end:
                break
        if not ok:
            return False, reason
        cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        for _ in range(5):
            ok, reason = self._acked(cmd, 0, 0, 0, 0, 0, 0, altitude)
            if ok:
                logger.info("takeoff accepted, %.1f m", altitude)
                return True, ""
            if self.abort.wait(2.0):    # the pause between tries, unless cut short
                break
        return False, "NAV_TAKEOFF not accepted"

    def land(self):
        return self.set_mode("LAND")

    def return_to_home(self):
        return self.set_mode("RTL")

    def hold(self):
        if not self.in_air():
            return True, ""
        return self.set_mode("BRAKE")

    def prepare_sticks(self):
        # GUIDED is the mode that takes velocity setpoints
        if self.armed() and self.mode_name() != "GUIDED":
            self.set_mode("GUIDED", timeout=5.0)

    def release_sticks(self):
        self.send_velocity_body(0, 0, 0, 0)
