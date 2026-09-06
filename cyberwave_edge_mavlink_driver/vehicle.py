"""What every autopilot can do. Each verb returns (ok, reason)."""

import logging
import time

from pymavlink import mavutil

logger = logging.getLogger(__name__)

# MAV_CMD_COMPONENT_ARM_DISARM param2: arm or disarm anyway, skip the checks
FORCE_ARM = 2989
FORCE_DISARM = 21196


def result_name(result):
    """A MAV_RESULT number as its enum name."""
    return getattr(mavutil.mavlink.enums["MAV_RESULT"].get(result),
                   "name", f"MAV_RESULT {result}")


class Vehicle:
    """The verbs both autopilots share. Subclasses add the rest."""

    name = "unknown"

    def __init__(self, link):
        self.link = link

    def armed(self):
        return bool(self.link.state["armed"])

    def in_air(self):
        """The autopilot's landed state, else the relative altitude."""
        landed = self.link.state["landed"]
        if landed is not None:
            return landed == "in_air"
        return self.link.state["alt"] > 0.5

    def set_armed(self, arm, force=False, timeout=5.0):
        """Arm or disarm, then wait for the heartbeat to agree.

        On refusal the reason is the autopilot's own words from STATUSTEXT,
        or the COMMAND_ACK result if it said nothing.
        """
        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        word = "arm" if arm else "disarm"
        p2 = (FORCE_ARM if arm else FORCE_DISARM) if force else 0
        t0 = time.time()
        self.link.send_command(cmd, 1 if arm else 0, p2)
        logger.info("%s sent (force=%s)", word, force)

        while time.time() < t0 + timeout:
            if self.link.state["armed"] == arm:
                return True, ""
            time.sleep(0.05)

        texts = self.link.texts_since(t0)
        texts = [t for t in texts if t.lower().startswith(("arm", "prearm"))] or texts
        result = self.link.state["acks"].pop(cmd, None)
        if texts:
            reason = "; ".join(dict.fromkeys(texts))
        elif result is not None:
            reason = result_name(result)
        else:
            reason = f"no armed-state change and no COMMAND_ACK within {timeout:.1f}s"
        logger.warning("%s refused: %s", word, reason)
        return False, reason

    def kill(self):
        """Motors off now, whatever the aircraft is doing."""
        logger.warning("force disarm sent")
        return self.set_armed(False, force=True, timeout=3.0)

    def set_home_here(self):
        return self._acked(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 1)

    def reboot(self):
        if self.armed():
            return False, "refused: the aircraft is armed"
        return self._acked(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        self.link.send_velocity_body(vx, vy, vz, yaw_rate)

    def _acked(self, cmd, *params, timeout=5.0):
        """Send one command and turn its ack into (ok, reason)."""
        self.link.send_command(cmd, *params)
        result = self.link.wait_ack(cmd, timeout)
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return True, ""
        if result is None:
            return False, f"no COMMAND_ACK within {timeout:.1f}s"
        return False, result_name(result)

    # -- per autopilot ---------------------------------------------------

    def mode_name(self):
        raise NotImplementedError

    def returning(self):
        raise NotImplementedError

    def takeoff(self, altitude):
        raise NotImplementedError

    def land(self):
        raise NotImplementedError

    def return_to_home(self):
        raise NotImplementedError

    def hold(self):
        raise NotImplementedError

    def prepare_sticks(self):
        """Put the aircraft in a mode that takes velocity setpoints."""
        raise NotImplementedError

    def release_sticks(self):
        """Stop the aircraft once the stick stream has ended."""
        raise NotImplementedError


def pick_vehicle(link):
    """The backend for the autopilot that answered the heartbeat."""
    from .ardupilot import ArduPilot
    from .px4 import PX4

    if link.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
        return PX4(link)
    if link.autopilot != mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        logger.warning("autopilot %s is not one we know, treating it as ArduPilot",
                       link.autopilot)
    return ArduPilot(link)
