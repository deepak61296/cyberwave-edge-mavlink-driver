"""The drone command vocabulary this driver speaks."""

# One-shot commands. Each one answers on the command topic.
DISCRETE = (
    "takeoff", "land", "return_to_home",
    "cancel_takeoff", "cancel_landing", "cancel_return_to_home",
    "emergency_stop", "set_home_here", "set_home_location",
    "reboot", "reboot_aircraft",
    "arm", "disarm", "brake", "kill",
    "hover",    # what the SDK's flight handle sends; another name for brake
    "gimbal_rotate", "set_gimbal_pitch", "gimbal_rotate_speed",
    "start_compass_calibration", "stop_compass_calibration",
)

# One verb, two names: the DJI catalog carries both and means the same thing.
REBOOT = ("reboot", "reboot_aircraft")

# Refusals whose words the contract fixes, so every driver says them the same.
NOT_SUPPORTED = "not supported on this vehicle"
MOTORS_RUNNING = "motors running"

# These do not queue: whatever discrete verb is running gives way to them.
# A cancel that waited for the verb it cancels would hold at the altitude it
# was sent to stop, so the cancels are here with the holds they are named for.
# stop is not one of them. The SDK ends every stick burst with a stop, so a
# verb sent during a burst would be cancelled by the burst's own tail.
URGENT = ("kill", "brake", "emergency_stop", "hover",
          "cancel_takeoff", "cancel_landing", "cancel_return_to_home")

# Nothing to land, cancel or hold on a parked aircraft: these are refused
# "not in air" while it is on the ground with the motors off. With the motors
# running a hold is still a real mode change, so armed on the ground is fine.
NEEDS_AIR = ("land", "return_to_home", "cancel_takeoff", "cancel_landing",
             "cancel_return_to_home", "brake", "hover", "emergency_stop")

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

# Each continuous verb in the words the catalog uses, and the payload field
# named for the axis it moves along. The SDK sends linear_x or angular_z
# whatever the verb is, so both that field and this one are read.
STICKS = {
    "move_forward":  ("Fly forward", "linear_x"),
    "move_backward": ("Fly backward", "linear_x"),
    "strafe_right":  ("Slide right", "linear_y"),
    "strafe_left":   ("Slide left", "linear_y"),
    "descend":       ("Descend", "linear_z"),
    "ascend":        ("Climb", "linear_z"),
    "turn_right":    ("Yaw right", "angular_z"),
    "turn_left":     ("Yaw left", "angular_z"),
}

# Camera sticks. Same dead-man as the flight sticks, but they move the
# gimbal and not the aircraft, so they are live on the ground too.
GIMBAL_STICKS = {
    "gimbal_pitch_up":   ("Tilt the camera up", 1),
    "gimbal_pitch_down": ("Tilt the camera down", -1),
}

# every verb the driver treats as a stick, the aircraft's and the camera's
STICK_VERBS = tuple(CONTINUOUS) + tuple(GIMBAL_STICKS)

STICK_TIMEOUT_S = 0.5      # no refresh in this long and the sticks release
MAX_TRAVEL_S = 30.0        # the longest a distance may keep the sticks live
DEFAULT_SPEED = 1.0        # m/s when a stick command carries no magnitude
DEFAULT_YAW_RATE = 0.5     # rad/s
DEFAULT_TAKEOFF_ALT = 2.0  # m
DEFAULT_GIMBAL_RATE = 30.0  # deg/s when a gimbal stick carries no rate
SPEED_UNIT_PER_DPS = 10.0   # the SDK sends gimbal rates in 0.1 deg/s

# What the catalog is told about a verb: command -> (description, args), an
# argument being (name, default, unit). define_interface turns these into the
# SDK's CommandArg, which is what the platform and the MCP read. A verb that
# takes nothing still has a line here, for its description.
CATALOG = {
    "takeoff": ("Arm and climb to the given altitude",
                (("altitude", DEFAULT_TAKEOFF_ALT, "m"),)),
    "land": ("Land where the aircraft is", ()),
    "return_to_home": ("Fly back to the home point and land", ()),
    "cancel_takeoff": ("Stop the climb and hold position", ()),
    "cancel_landing": ("Stop the descent and hold position", ()),
    "cancel_return_to_home": ("Stop the return and hold position", ()),
    "emergency_stop": ("Drop the automation and hold position, motors running", ()),
    "set_home_here": ("Make the current position home", ()),
    "reboot": ("Reboot the flight controller, refused with the motors running", ()),
    "arm": ("Start the motors; force skips the autopilot's own checks",
            (("force", False, None),)),
    "disarm": ("Stop the motors; in the air it takes force",
               (("force", False, None),)),
    "brake": ("Stop moving and hold position", ()),
    "kill": ("Cut the motors now; in the air it takes force",
             (("force", False, None),)),
    "hover": ("Hold position, the flight handle's word for brake", ()),
    "set_home_location": ("Make the given point home",
                          (("latitude", None, "deg"), ("longitude", None, "deg"),
                           ("altitude", None, "m"))),
    "reboot_aircraft": ("Reboot the flight controller, the catalog's other "
                        "name for reboot", ()),
    "gimbal_rotate": ("Point the camera, by mode absolute or relative to where "
                      "it is now",
                      (("pitch", None, "deg"), ("yaw", None, "deg"),
                       ("roll", None, "deg"), ("mode", "absolute", None),
                       ("duration", None, "s"))),
    "set_gimbal_pitch": ("Point the camera at an absolute pitch",
                         (("pitch", 0.0, "deg"),)),
    "gimbal_rotate_speed": ("Turn the camera at a rate until it is stopped",
                            (("pitch", None, "0.1 deg/s"),
                             ("yaw", None, "0.1 deg/s"),
                             ("roll", None, "0.1 deg/s"))),
    "start_compass_calibration": ("Begin compass calibration, refused with the "
                                  "motors running", ()),
    "stop_compass_calibration": ("Abort a compass calibration", ()),
}

# Every stick verb takes the same two: how fast, and how far.
for _verb, (_words, _axis) in STICKS.items():
    _turn = _axis == "angular_z"
    CATALOG[_verb] = (
        f"{_words} while the bursts keep coming, or by the distance given",
        ((_axis, DEFAULT_YAW_RATE if _turn else DEFAULT_SPEED,
          "rad/s" if _turn else "m/s"),
         ("distance", None, "rad" if _turn else "m")))

# The camera sticks take a rate and, like the others, an optional angle to
# travel. Their axis is gimbal pitch, so both are in degrees.
for _verb, (_words, _sign) in GIMBAL_STICKS.items():
    CATALOG[_verb] = (
        f"{_words} while the bursts keep coming, or by the angle given",
        (("rate", DEFAULT_GIMBAL_RATE, "deg/s"), ("distance", None, "deg")))


def flight_state(connected, armed, in_air, returning, was_airborne):
    """One word for where the aircraft is in a flight."""
    if not connected:
        return "disconnected"
    if not armed:
        return "ready"
    if in_air:
        return "returning" if returning else "in_air"
    return "landed" if was_airborne else "motors_on"
