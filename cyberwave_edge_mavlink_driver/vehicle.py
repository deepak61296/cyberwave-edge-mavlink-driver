"""What every autopilot can do. Each verb returns (ok, reason)."""

import logging
import threading
import time

from pymavlink import mavutil

from .contract import MOTORS_RUNNING, NOT_SUPPORTED
from .link import BODY_OFFSET_NED

logger = logging.getLogger(__name__)

# MAV_CMD_COMPONENT_ARM_DISARM param2: arm or disarm anyway, skip the checks
FORCE_ARM = 2989
FORCE_DISARM = 21196

# An angle or a rate given as NaN is an axis the caller did not command, which
# is MAVLink's own convention and the contract's "an axis left out".
NAN = float("nan")


class Refused(Exception):
    """A verb the vehicle will not do, in the contract's own words.

    The verbs that answer with (ok, reason) keep doing that. The ones that
    return nothing say no by raising this, and the driver puts the message
    straight into the reply's reason.
    """


def result_name(result):
    """A MAV_RESULT number as its enum name."""
    return getattr(mavutil.mavlink.enums["MAV_RESULT"].get(result),
                   "name", f"MAV_RESULT {result}")


class Vehicle:
    """The verbs both autopilots share. Subclasses add the rest."""

    name = "unknown"
    velocity_frame = BODY_OFFSET_NED

    def __init__(self, link):
        self.link = link
        # kill, brake, emergency_stop and stop set this; every wait gives way to it
        self.abort = threading.Event()

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

        if self._wait(lambda: self.link.state["armed"] == arm, timeout):
            return True, ""

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

    def takeoff_altitude(self, asked):
        """What the takeoff reply reports as altitude_m.

        The altitude we asked for, unless the backend confirmed the climb
        against a real reading and can say how high the aircraft got.
        """
        return asked

    def kill(self):
        """Motors off now, whatever the aircraft is doing."""
        logger.warning("force disarm sent")
        return self.set_armed(False, force=True, timeout=3.0)

    def set_home_here(self):
        return self._acked(mavutil.mavlink.MAV_CMD_DO_SET_HOME, 1)

    def reboot(self):
        if self.armed():
            return False, MOTORS_RUNNING
        return self._acked(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        self.link.send_velocity_body(vx, vy, vz, yaw_rate, self.velocity_frame)

    def tick(self):
        """Called every driver tick. Nothing to keep up by default."""

    def _wait(self, predicate, timeout, meanwhile=None):
        """Poll predicate until it holds, the timeout passes or abort is set.

        meanwhile, if given, runs on every poll: for a stream that must not
        stop while we wait.
        """
        end = time.time() + timeout
        while True:
            if predicate():
                return True
            if meanwhile is not None:
                meanwhile()
            if time.time() >= end or self.abort.wait(0.05):
                return False

    def _acked(self, cmd, *params, timeout=5.0):
        """Send one command and turn its ack into (ok, reason)."""
        self.link.send_command(cmd, *params)
        acks = self.link.state["acks"]
        if not self._wait(lambda: cmd in acks, timeout):
            return False, f"no COMMAND_ACK within {timeout:.1f}s"
        result = acks.pop(cmd)
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            return True, ""
        return False, result_name(result)

    def _set_mode(self, name, want, params, timeout, fill=0.0, meanwhile=None):
        """DO_SET_MODE(*params) once a second until the heartbeat reads want."""
        t0 = time.time()

        def in_mode():
            return self.link.state["mode"] == want

        while not in_mode():
            if self.abort.is_set() or time.time() > t0 + timeout:
                # the autopilot usually says why on the text channel; prefer its words
                why = "; ".join(dict.fromkeys(self.link.texts_since(t0)))
                logger.error("could not enter %s: %s", name, why)
                return False, why or f"could not enter {name} within {timeout:.0f}s"
            self.link.send_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                   mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                   *params, fill=fill)
            self._wait(in_mode, min(1.0, t0 + timeout - time.time()), meanwhile)
        return True, ""

    # -- camera, home and compass ----------------------------------------
    #
    # Everything a vehicle without a gimbal, or without these commands, can
    # leave exactly as it is: the refusal is already the contract's phrase.

    def gimbal_point(self, pitch_deg, yaw_deg, absolute, duration_s=None):
        """Point the camera at pitch and yaw, in degrees.

        absolute False makes both a delta from where the gimbal is now. An
        axis given as NAN is one the caller did not command. duration_s, when
        given, is how long the move should take.
        """
        raise Refused(NOT_SUPPORTED)

    def gimbal_rate(self, pitch_dps, yaw_dps):
        """Turn the camera at these rates, in degrees per second.

        Called from the tick on every pass while a gimbal stick is live, and
        once with zeros when it is released, so it must not wait on anything.
        """
        raise Refused(NOT_SUPPORTED)

    def gimbal_attitude(self):
        """(pitch, yaw) in degrees, or None when there is no gimbal to read."""
        return None

    def set_home(self, lat, lon, alt_m):
        """Record this point as home. alt_m None keeps the height home has."""
        raise Refused(NOT_SUPPORTED)

    def compass_calibration(self, start):
        """Begin the compass calibration, or abort the one running."""
        raise Refused(NOT_SUPPORTED)

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
