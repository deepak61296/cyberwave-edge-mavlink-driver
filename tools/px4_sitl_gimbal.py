#!/usr/bin/env python3
"""Give PX4 SITL a gimbal, over the GCS port. SITL only.

The gimbal module is off by default (MNT_MODE_IN -1) and rcS starts it only
when that param is set, so a fresh sihsim has no gimbal manager at all and
DO_GIMBAL_MANAGER_PITCHYAW is never even acked. In AUX output mode the module
also refuses to act as a gimbal device until some output channel carries a
gimbal function, so the three PWM_MAIN_FUNC lines below are what make
GIMBAL_DEVICE_ATTITUDE_STATUS appear.

Both changes need a restart: kill the SITL with SIGINT and start it again on
the same -w directory, which keeps parameters.bson.

    python tools/px4_sitl_gimbal.py            # set them
    python tools/px4_sitl_gimbal.py --show     # read them back
"""

import argparse
import struct
import sys
import time

from pymavlink import mavutil

GCS_PORT = "udpin:127.0.0.1:14550"

# name -> (value, is int)
PARAMS = (
    ("MNT_MODE_IN", 4, True),     # MAVLink gimbal protocol v2
    ("MNT_MODE_OUT", 0, True),    # AUX, the simulated servo output
    ("PWM_MAIN_FUNC5", 420, True),   # Gimbal_Roll
    ("PWM_MAIN_FUNC6", 421, True),   # Gimbal_Pitch
    ("PWM_MAIN_FUNC7", 422, True),   # Gimbal_Yaw
    ("MNT_DO_STAB", 0, True),     # no stabilisation; sihsim has no real mount
    # MNT_TAU models a servo's lag, and in AUX mode it is applied to the angle
    # the module integrates, not only to the output: at the 0.3 s default a
    # 10 deg/s rate moves the mount about 0.4 deg/s. Zero for a bench.
    ("MNT_TAU", 0.0, False),
)


def as_float_bits(value):
    """PX4 carries an INT32 param in the float field's bits, not its value."""
    return struct.unpack("<f", struct.pack("<i", int(value)))[0]


def from_float_bits(value):
    return struct.unpack("<i", struct.pack("<f", value))[0]


def read(m, name, is_int, timeout=3.0):
    m.mav.param_request_read_send(m.target_system, m.target_component,
                                  name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id.strip("\x00") == name:
            return from_float_bits(msg.param_value) if is_int else msg.param_value
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--show", action="store_true", help="read, do not write")
    args = ap.parse_args()

    m = mavutil.mavlink_connection(GCS_PORT)
    m.wait_heartbeat()
    print(f"heartbeat from sys {m.target_system} comp {m.target_component}")

    for name, value, is_int in PARAMS:
        if not args.show:
            kind = (mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int
                    else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            m.mav.param_set_send(m.target_system, m.target_component, name.encode(),
                                 as_float_bits(value) if is_int else float(value), kind)
            time.sleep(0.1)
        got = read(m, name, is_int)
        mark = "ok" if got == value else "NOT SET"
        print(f"{name:16} {got}   ({mark})")

    if not args.show:
        print("\nnow restart the SITL (SIGINT, then start it again on the same -w dir)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
