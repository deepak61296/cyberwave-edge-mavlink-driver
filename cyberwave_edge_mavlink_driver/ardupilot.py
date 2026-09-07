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
        by_number = {v: k for k, v in self.link.mode_names().items()}
        mode = self.link.state["mode"]
        return by_number.get(mode, f"MODE({mode})")

    def returning(self):
        return self.mode_name() in ("RTL", "SMART_RTL")

    def set_mode(self, name, timeout=30.0):
        """Ask for a mode once a second until the heartbeat shows it."""
        want = self.link.mode_names().get(name)
        if want is None:
            return False, f"unknown mode {name}"
        t0 = time.time()
        next_send = 0.0
        while time.time() < t0 + timeout:
            if self.link.state["mode"] == want:
                return True, ""
            if time.time() >= next_send:
                self.link.m.set_mode(name)
                next_send = time.time() + 1.0
            time.sleep(0.1)
        # the autopilot usually says why on the text channel; prefer its words
        why = "; ".join(dict.fromkeys(self.link.texts_since(t0)))
        logger.error("could not enter mode %s: %s", name, why)
        return False, why or f"could not enter {name} within {timeout:.0f}s"

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
            if ok or time.time() > end:
                break
        if not ok:
            return False, reason
        cmd = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        for _ in range(5):
            self.link.send_command(cmd, 0, 0, 0, 0, 0, 0, altitude)
            if self.link.wait_ack(cmd) == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("takeoff accepted, %.1f m", altitude)
                return True, ""
            time.sleep(2.0)
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
