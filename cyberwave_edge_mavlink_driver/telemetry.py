"""What the twin is told: pose, prop spin and a small vehicle_state record."""

import math
import time

from . import contract

STATE_PERIOD_S = 1.0    # vehicle_state goes out at least this often

# Prop-joint animation (twin assets with continuous prop joints).
# The viewer renders joint POSITIONS only, so we integrate a PWM-scaled
# visual spin rate into wrapped angles: legible spin, not true prop RPM.
# These are the px4vision names, the fallback for a twin that lists none;
# the driver asks the twin for its own at start.
PROP_JOINTS = ("prop_1_joint", "prop_2_joint", "prop_3_joint", "prop_4_joint")
# Direction goes by position in the list, which for the px4vision asset is
# its URDF order (prop_1/2 carry the CCW mesh, prop_3/4 the CW mesh) and
# also ArduPilot's quad-X motor order: M1 front-right CCW, M2 rear-left
# CCW, M3 front-left CW, M4 rear-right CW. CCW from above = positive +Z.
PROP_DIRS = (1, 1, -1, -1)
PROP_VISUAL_MAX_RAD_S = 60.0  # idle (1100 us) = 6 rad/s: ~35 deg per 10 Hz update, no wagon-wheel reversal


def prop_joint_names(names):
    """The prop joints among a twin's joint names, sorted so the spin
    directions land the same way on every start."""
    return tuple(sorted(n for n in names if "prop" in n.lower()))


class PropSpin:
    """Prop angles, advanced from the latest SERVO_OUTPUT_RAW."""

    def __init__(self, joints=PROP_JOINTS):
        # position picks the direction and the PWM channel, and there are
        # four of each, so a longer list loses its tail
        self.joints = tuple(joints)[:len(PROP_DIRS)]
        self.angles = [0.0] * len(self.joints)
        self.at = time.time()

    def payload(self, pwm):
        """The joint/update payload for this instant, or None without PWM."""
        if pwm is None:
            return None
        now = time.time()
        dt = min(now - self.at, 0.5)
        self.at = now
        positions, velocities = {}, {}
        for i, name in enumerate(self.joints):
            omega = 0.0
            # sanity-bounded: unused channels report 0, and a raw 65535
            # (UINT16 "unknown") must not read as a full-speed prop
            if pwm[i] and 1050 < pwm[i] <= 2200:
                omega = PROP_DIRS[i] * PROP_VISUAL_MAX_RAD_S * \
                    min((pwm[i] - 1000) / 1000.0, 1.0)
            self.angles[i] = (self.angles[i] + omega * dt) % (2 * math.pi)
            positions[name] = self.angles[i]
            velocities[name] = omega
        return {"positions": positions, "velocities": velocities,
                "source_type": "edge", "timestamp": now}


class Telemetry:
    """Payloads for the twin's topics, built from the link's state cache."""

    def __init__(self, link):
        self.link = link
        self.vehicle = None         # set once the autopilot has answered
        self.props = PropSpin()
        self.was_airborne = False
        self._state_last = None
        self._state_at = 0.0

    def flight_state(self):
        if self.vehicle is None:
            return "disconnected"
        armed, in_air = self.vehicle.armed(), self.vehicle.in_air()
        # motors off ends the flight, so the next one starts from ready again
        self.was_airborne = armed and (self.was_airborne or in_air)
        return contract.flight_state(self.link.connected(), armed, in_air,
                                     self.vehicle.returning(), self.was_airborne)

    def position(self):
        # the state cache keeps its last fix for ever, and a stale pose
        # republished at 10 Hz reads as a live aircraft
        if not self.link.connected():
            return None
        pos = self.link.position_enu()
        if pos is None:
            return None
        return {"type": "position", "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
                "source_type": "edge", "timestamp": time.time()}

    def rotation(self):
        if not self.link.connected():
            return None
        quat = self.link.attitude_quat_enu()
        if quat is None:
            return None
        return {"type": "rotation", "rotation": quat,
                "source_type": "edge", "timestamp": time.time()}

    def use_prop_joints(self, names):
        """Spin the joints the twin's own asset has, not the px4vision ones."""
        self.props = PropSpin(names)

    def prop_joints(self):
        return self.props.payload(self.link.state["servo_pwm"])

    def vehicle_state(self):
        """Armed, mode, flight state and the camera angle: on change and at
        least once a second. A vehicle with no gimbal sends the fields at all."""
        if self.vehicle is None:
            return None
        now = time.time()
        gimbal = self.vehicle.gimbal_attitude()
        snapshot = (self.vehicle.armed(), self.vehicle.mode_name(), self.flight_state(),
                    None if gimbal is None else tuple(round(a, 1) for a in gimbal))
        if snapshot == self._state_last and now - self._state_at < STATE_PERIOD_S:
            return None
        self._state_last, self._state_at = snapshot, now
        pwm = self.link.state["servo_pwm"]
        payload = {"type": "vehicle_state", "armed": snapshot[0], "mode": snapshot[1],
                   "flight_state": snapshot[2], "motors_pwm": list(pwm) if pwm else None,
                   "source_type": "edge", "timestamp": now}
        if gimbal is not None:
            payload["gimbal_pitch"] = round(gimbal[0], 2)
            payload["gimbal_yaw"] = round(gimbal[1], 2)
        return payload

    def summary(self):
        """The fields merged into the driver's own telemetry snapshots."""
        if self.vehicle is None:
            return {}
        return {"autopilot": self.vehicle.name, "armed": self.vehicle.armed(),
                "mode": self.vehicle.mode_name(), "flight_state": self.flight_state()}
