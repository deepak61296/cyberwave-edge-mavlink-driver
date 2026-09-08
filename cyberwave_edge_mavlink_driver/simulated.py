"""An aircraft that flies inside this process. No autopilot, no socket.

MAVLINK_CONNECTION=sim://quad or sim://dji picks it. The link is the same
state cache the pump would fill, written by the model on every tick, and
the model is a small kinematic one: body velocities integrate into
position and heading, altitude climbs and descends at fixed rates. It is
here so the command contract can be exercised in CI and in a demo with no
SITL running, and so the DJI Mini 4 Pro's answers, which no software
simulator can produce, have one runnable reference.
"""

import collections
import logging
import math
import time

from . import contract
from .link import MavlinkLink
from .vehicle import Refused, Vehicle

logger = logging.getLogger(__name__)

SIM_PREFIX = "sim://"

CLIMB_MPS = 1.5         # takeoff
DESCEND_MPS = 1.0       # land
FALL_MPS = 6.0          # motors cut in the air
CRUISE_MPS = 5.0        # flying home
GROUND_M = 0.05         # below this the aircraft is on the ground
ARRIVED_M = 0.1         # close enough to the target altitude, or to home
TAKEOFF_S = 30.0        # how long takeoff waits for the climb
LAND_CONFIRM_M = 0.7    # where a DJI landing parks and asks the operator
DJI_TAKEOFF_M = 1.2     # DJI climbs to this whatever altitude was asked for
TILT_RAD_PER_MPS = 0.1  # how far the model leans into the direction it flies
MAX_TILT_RAD = 0.35
GIMBAL_SNAP_DPS = 400.0  # a point move with no duration, near enough instant

ZERO = (0.0, 0.0, 0.0, 0.0)

# mode_name(), keyed by what the model is doing. "ground" is parked.
QUAD_MODES = {None: "GUIDED", "ground": "STABILIZE", "takeoff": "GUIDED",
              "land": "LAND", "rtl": "RTL", "sticks": "GUIDED"}
DJI_MODES = {None: "HOVER", "ground": "READY", "takeoff": "TAKE_OFF",
             "land": "AUTO_LANDING", "rtl": "GO_HOME", "sticks": "VIRTUAL_STICK"}

# gimbal is (pitch, yaw) limits in degrees; None is an axis that does not steer
Profile = collections.namedtuple(
    "Profile", "name implicit_arm can_kill takeoff_m confirm gimbal modes")

QUAD = Profile("quad", False, True, None, False,
               ((-90.0, 30.0), (-160.0, 160.0)), QUAD_MODES)
# The Mini 4 Pro as docs/DJI-MAPPING.md describes it: no arming key, no motor
# cut, a fixed takeoff height, an operator confirm on land and go home, and a
# gimbal whose yaw is not independently steerable.
DJI = Profile("dji", True, False, DJI_TAKEOFF_M, True,
              ((-90.0, 60.0), None), DJI_MODES)

PROFILES = {"quad": QUAD, "dji": DJI}


def profile_for(connection):
    """The profile named by a sim:// connection string."""
    name = connection[len(SIM_PREFIX):]
    if name not in PROFILES:
        logger.warning("no sim profile %r, flying the quad", name)
    return PROFILES.get(name, QUAD)


def clamp(value, low, high):
    return min(max(value, low), high)


def toward(value, want, step):
    """value moved at most step towards want."""
    return value + clamp(want - value, -step, step)


class SimLink(MavlinkLink):
    """The link with nothing behind it: the same state cache, no socket."""

    def __init__(self, connection):
        super().__init__(connection)
        self.open = False

    def connect(self, timeout=60.0):
        self.open = True
        self.state["last_heartbeat"] = time.time()
        logger.info("simulated aircraft on %s", self.connection_string)

    def connected(self):
        return self.open

    def ready(self):
        return self.open

    def close(self):
        self.open = False

    def pump_once(self, timeout=1.0):
        """Nothing arrives here. The driver's pump thread waits instead."""
        time.sleep(timeout)

    def request_streams(self):
        pass


class SimVehicle(Vehicle):
    """A drone that only exists here. Every verb answers the way its profile
    says the real aircraft would; tick() is what actually flies it."""

    def __init__(self, link, now=time.time):
        super().__init__(link)
        self.profile = profile_for(link.connection_string)
        self.name = "sim-" + self.profile.name
        self.now = now              # a test hands in a clock it can move
        self.at = now()
        self.ned = [0.0, 0.0, 0.0]  # north, east, down in metres
        self.yaw = 0.0              # radians, 0 is north
        self.home = [0.0, 0.0]      # where return_to_home flies to
        self.home_fix = None        # home as coordinates, when one was given
        self.task = None            # takeoff, land, rtl, sticks or None
        self.target_m = 0.0         # the altitude a takeoff climbs to
        self.pending = None         # the verb the operator has been asked about
        self.calibrating = False
        self.stick = ZERO
        self.stick_at = 0.0
        self.gimbal = [0.0, 0.0]    # pitch, yaw in degrees
        self.aim = None             # gimbal angles being moved to
        self.aim_dps = GIMBAL_SNAP_DPS
        self.gimbal_dps = (0.0, 0.0)
        self.gimbal_at = 0.0
        self._publish()

    def alt(self):
        return max(0.0, -self.ned[2])

    def mode_name(self):
        if self.task is None and not self.in_air():
            return self.profile.modes["ground"]
        return self.profile.modes[self.task]

    def returning(self):
        return self.task == "rtl"

    # -- verbs -----------------------------------------------------------

    def set_armed(self, arm, force=False, timeout=5.0):
        if self.profile.implicit_arm:
            # no arming key here: the motors start with the takeoff and stop
            # with the landing, so on the ground the verb is an ok that does
            # nothing. In the air there is no key to stop them with either.
            if not arm and self.in_air():
                return False, contract.NOT_SUPPORTED
            return True, "", {"implicit": True}
        self.link.state["armed"] = bool(arm)
        if not arm:
            self.task, self.pending, self.stick = None, None, ZERO
        return True, ""

    def kill(self):
        if not self.profile.can_kill:
            return False, contract.NOT_SUPPORTED
        logger.warning("motors cut")
        return self.set_armed(False, force=True)

    def takeoff(self, altitude):
        if self.in_air():
            return False, "already in air"
        # DJI climbs to its own height, so there the altitude asked for is
        # a request and nothing more
        self.target_m = self.profile.takeoff_m or float(altitude)
        self.link.state["armed"] = True
        self.pending, self.task = None, "takeoff"
        if self._wait(lambda: self.task != "takeoff", TAKEOFF_S):
            logger.info("airborne at %.1f m", self.alt())
            return True, ""
        return False, f"still climbing, {self.alt():.1f} m of {self.target_m:.1f} m"

    def takeoff_altitude(self, asked):
        # takeoff returns once the climb is done, so this is the real height
        return round(self.alt(), 2) if self.in_air() else asked

    def land(self):
        first = self.task != "land"
        self.task = "land"
        if self.profile.confirm and first:
            # DJI parks at 0.7 m and waits; a second land sends the confirm
            self.pending = "land"
            return True, "", {"pending_confirmation": True}
        self.pending = None
        return True, ""

    def return_to_home(self):
        if self.profile.confirm and self.pending != "rtl":
            # the aircraft holds where it is until a second return_to_home
            self.pending = "rtl"
            return True, "", {"pending_confirmation": True}
        self.pending, self.task = None, "rtl"
        return True, ""

    def hold(self):
        """Cancel whatever is running and stop where we are."""
        self.task, self.pending, self.stick = None, None, ZERO
        return True, ""

    def set_home_here(self):
        self.home = [self.ned[0], self.ned[1]]
        return True, ""

    def set_home(self, lat, lon, alt_m):
        """Record a home point given as coordinates.

        The model flies in metres from where it started, so the point is kept
        as it arrived and return_to_home still flies to the local home.
        """
        height = self.home_fix[2] if alt_m is None and self.home_fix else alt_m
        self.home_fix = (float(lat), float(lon), height)

    def reboot(self):
        if self.armed():
            return False, contract.MOTORS_RUNNING
        self.task, self.pending, self.stick = None, None, ZERO
        return True, ""

    def compass_calibration(self, start):
        self.calibrating = bool(start)

    def prepare_sticks(self):
        if self.armed() and self.task is None:
            self.task = "sticks"

    def release_sticks(self):
        self.stick = ZERO
        if self.task == "sticks":
            self.task = None

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        """One setpoint from the driver's stream. The model flies the last
        one it was given until the stream stops."""
        self.stick = (vx, vy, vz, yaw_rate)
        self.stick_at = self.now()

    # -- gimbal ----------------------------------------------------------

    def gimbal_point(self, pitch_deg, yaw_deg, absolute, duration_s=None):
        """Aim the gimbal, in degrees. A NaN axis is one nobody asked for."""
        target = list(self.aim or self.gimbal)
        for axis, asked in enumerate((pitch_deg, yaw_deg)):
            if asked != asked:
                continue
            target[axis] = clamp(asked if absolute else target[axis] + asked,
                                 *self._limits(axis))
        far = max(abs(t - g) for t, g in zip(target, self.gimbal))
        self.aim = target
        self.aim_dps = far / duration_s if duration_s else GIMBAL_SNAP_DPS
        self.gimbal_dps = (0.0, 0.0)

    def gimbal_rate(self, pitch_dps, yaw_dps):
        """Turn the gimbal until it is zeroed, stops refreshing or hits a stop."""
        rates = []
        for axis, asked in enumerate((pitch_dps, yaw_dps)):
            asked = 0.0 if asked != asked else float(asked)
            if asked:
                self._limits(axis)      # an axis that does not steer refuses
            rates.append(asked)
        self.gimbal_dps = tuple(rates)
        self.gimbal_at = self.now()
        self.aim = None

    def _limits(self, axis):
        """How far this gimbal axis goes, or a refusal if it does not turn."""
        limits = self.profile.gimbal[axis]
        if limits is None:
            raise Refused(contract.NOT_SUPPORTED)
        return limits

    def gimbal_attitude(self):
        """Pitch and yaw in degrees, or None on a vehicle without a gimbal."""
        if not any(self.profile.gimbal):
            return None
        return self.gimbal[0], self.gimbal[1]

    # -- the model -------------------------------------------------------

    def tick(self):
        now = self.now()
        dt = min(max(now - self.at, 0.0), 0.5)
        self.at = now
        self._fly(dt)
        self._turn_gimbal(dt)
        self._publish()

    def _fly(self, dt):
        if not self.armed():
            return self._drop(dt)
        if self.task == "takeoff":
            self._climb(dt)
        elif self.task == "land":
            self._land(dt)
        elif self.task == "rtl":
            self._go_home(dt)
        else:
            self._sticks(dt)

    def _drop(self, dt):
        """Motors off in the air is not a hover, it is a fall."""
        self.ned[2] = min(self.ned[2] + FALL_MPS * dt, 0.0)
        self.task, self.stick = None, ZERO

    def _climb(self, dt):
        self.ned[2] = max(self.ned[2] - CLIMB_MPS * dt, -self.target_m)
        if self.alt() >= self.target_m:
            self.task = None

    def _land(self, dt):
        floor = LAND_CONFIRM_M if self.pending == "land" else 0.0
        self.ned[2] = min(self.ned[2] + DESCEND_MPS * dt, -floor)
        if not floor and self.alt() <= GROUND_M:
            self._park()

    def _go_home(self, dt):
        """Fly to the home point, then down onto it."""
        north, east = self.home[0] - self.ned[0], self.home[1] - self.ned[1]
        far = math.hypot(north, east)
        if far > ARRIVED_M:
            step = min(CRUISE_MPS * dt, far)
            self.ned[0] += north * step / far
            self.ned[1] += east * step / far
            return
        self.ned[2] = min(self.ned[2] + DESCEND_MPS * dt, 0.0)
        if self.alt() <= GROUND_M:
            self._park()

    def _sticks(self, dt):
        if self.now() - self.stick_at > contract.STICK_TIMEOUT_S:
            self.stick = ZERO       # the dead man: no refresh, no motion
        vx, vy, vz, yaw_rate = self.stick
        self.yaw = (self.yaw + yaw_rate * dt) % (2 * math.pi)
        if not self.in_air():
            return                  # on the ground the sticks move nothing
        self.ned[0] += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.ned[1] += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.ned[2] = min(self.ned[2] + vz * dt, 0.0)

    def _turn_gimbal(self, dt):
        if self.aim is not None:
            step = self.aim_dps * dt
            self.gimbal = [toward(g, want, step)
                           for g, want in zip(self.gimbal, self.aim)]
        elif self.now() - self.gimbal_at <= contract.STICK_TIMEOUT_S:
            for axis, rate in enumerate(self.gimbal_dps):
                limits = self.profile.gimbal[axis] or (0.0, 0.0)
                self.gimbal[axis] = clamp(self.gimbal[axis] + rate * dt, *limits)

    def _park(self):
        """On the ground with the motors stopped, how both profiles land."""
        self.ned[2] = 0.0
        self.task, self.pending, self.stick = None, None, ZERO
        self.link.state["armed"] = False

    def _publish(self):
        """Fill the state cache the driver and the telemetry read."""
        s = self.link.state
        vx, vy = self.stick[0], self.stick[1]
        s["ned"] = tuple(self.ned)
        s["alt"] = self.alt()
        s["landed"] = "in_air" if self.alt() > GROUND_M else "on_ground"
        # a quad leans into the direction it flies, which is what the twin shows
        s["attitude"] = (clamp(vy * TILT_RAD_PER_MPS, -MAX_TILT_RAD, MAX_TILT_RAD),
                         clamp(-vx * TILT_RAD_PER_MPS, -MAX_TILT_RAD, MAX_TILT_RAD),
                         self.yaw)
        s["mode"] = self.mode_name()
        s["servo_pwm"] = (1100,) * 4 if self.armed() else (1000,) * 4
        # the driver's stream watchdog reads these against the wall clock
        s["last_attitude"] = s["last_heartbeat"] = time.time()
