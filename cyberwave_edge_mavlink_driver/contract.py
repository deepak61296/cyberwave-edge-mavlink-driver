"""The drone command vocabulary this driver speaks."""

# One-shot commands. Each one answers on the command topic.
DISCRETE = (
    "takeoff", "land", "return_to_home",
    "cancel_takeoff", "cancel_landing", "cancel_return_to_home",
    "emergency_stop", "set_home_here", "reboot",
    "arm", "disarm", "brake", "kill",
    "hover",    # what the SDK's flight handle sends; another name for brake
)

# These do not queue: whatever discrete verb is running gives way to them.
URGENT = ("kill", "brake", "emergency_stop", "stop")

# Stick commands: name -> body frame (vx, vy, vz, yaw_rate) unit vector.
# The body frame is NED, so +z is down.
CONTINUOUS = {
    "move_forward":  (1, 0, 0, 0),
    "move_backward": (-1, 0, 0, 0),
    "strafe_right":  (0, 1, 0, 0),
    "strafe_left":   (0, -1, 0, 0),
    "descend":       (0, 0, 1, 0),
    "ascend":        (0, 0, -1, 0),
    "turn_right":    (0, 0, 0, 1),
    "turn_left":     (0, 0, 0, -1),
}

STICK_TIMEOUT_S = 0.5      # no refresh in this long and the sticks release
DEFAULT_SPEED = 1.0        # m/s when a stick command carries no magnitude
DEFAULT_YAW_RATE = 0.5     # rad/s
DEFAULT_TAKEOFF_ALT = 2.0  # m


def flight_state(connected, armed, in_air, returning, was_airborne):
    """One word for where the aircraft is in a flight."""
    if not connected:
        return "disconnected"
    if not armed:
        return "ready"
    if in_air:
        return "returning" if returning else "in_air"
    return "landed" if was_airborne else "motors_on"
