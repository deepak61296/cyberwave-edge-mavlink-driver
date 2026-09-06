"""What the twin is told: pose, prop spin and a small vehicle_state record."""

import math
import time

from . import contract

STATE_PERIOD_S = 1.0    # vehicle_state goes out at least this often

# Prop-joint animation (twin assets with prop_N_joint continuous joints).
# The viewer renders joint POSITIONS only, so we integrate a PWM-scaled
# visual spin rate into wrapped angles: legible spin, not true prop RPM.
PROP_JOINTS = ("prop_1_joint", "prop_2_joint", "prop_3_joint", "prop_4_joint")
# Directions match the asset's URDF (prop_1/2 carry the CCW mesh, prop_3/4
# the CW mesh), which is also ArduPilot's quad-X motor order: M1 front-right
# CCW, M2 rear-left CCW, M3 front-left CW, M4 rear-right CW. CCW viewed
# from above = positive rotation about +Z.
PROP_DIRS = (1, 1, -1, -1)
PROP_VISUAL_MAX_RAD_S = 60.0  # idle (1100 us) = 6 rad/s: ~35 deg per 10 Hz update, no wagon-wheel reversal


class PropSpin:
    """Four prop angles, advanced from the latest SERVO_OUTPUT_RAW."""

    def __init__(self):
        self.angles = [0.0] * 4
        self.at = time.time()

    def payload(self, pwm):
        """The joint/update payload for this instant, or None without PWM."""
        if pwm is None:
            return None
        now = time.time()
        dt = min(now - self.at, 0.5)
        self.at = now
        positions, velocities = {}, {}
        for i, name in enumerate(PROP_JOINTS):
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
        in_air = self.vehicle.in_air()
        self.was_airborne = self.was_airborne or in_air
        return contract.flight_state(self.link.connected(), self.vehicle.armed(), in_air,
                                     self.vehicle.returning(), self.was_airborne)

    def position(self):
        pos = self.link.position_enu()
        if pos is None:
            return None
        return {"type": "position", "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
                "source_type": "edge", "timestamp": time.time()}

    def rotation(self):
        quat = self.link.attitude_quat_enu()
        if quat is None:
            return None
        return {"type": "rotation", "rotation": quat,
                "source_type": "edge", "timestamp": time.time()}

    def prop_joints(self):
        return self.props.payload(self.link.state["servo_pwm"])

    def vehicle_state(self):
        """Armed, mode and flight state: on change and at least once a second."""
        if self.vehicle is None:
            return None
        now = time.time()
        snapshot = (self.vehicle.armed(), self.vehicle.mode_name(), self.flight_state())
        if snapshot == self._state_last and now - self._state_at < STATE_PERIOD_S:
            return None
        self._state_last, self._state_at = snapshot, now
        pwm = self.link.state["servo_pwm"]
        return {"type": "vehicle_state", "armed": snapshot[0], "mode": snapshot[1],
                "flight_state": snapshot[2], "motors_pwm": list(pwm) if pwm else None,
                "source_type": "edge", "timestamp": now}

    def summary(self):
        """The fields merged into the driver's own telemetry snapshots."""
        if self.vehicle is None:
            return {}
        return {"autopilot": self.vehicle.name, "armed": self.vehicle.armed(),
                "mode": self.vehicle.mode_name(), "flight_state": self.flight_state()}
