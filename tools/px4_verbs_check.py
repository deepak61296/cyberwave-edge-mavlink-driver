#!/usr/bin/env python3
"""Drive the gimbal, home and compass verbs against a live PX4, no platform.

Everything goes through the driver's own MavlinkLink and PX4 classes, so what
this prints is what the driver would do; the only thing it adds is the pump
thread the driver normally runs. Point it at SITL:

    python tools/px4_verbs_check.py                 # on the ground
    python tools/px4_verbs_check.py --air           # takeoff, then again up there

SITL needs a gimbal first, see tools/px4_sitl_gimbal.py.
"""

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cyberwave_edge_mavlink_driver.link import MavlinkLink        # noqa: E402
from cyberwave_edge_mavlink_driver.px4 import PX4   # noqa: E402
from cyberwave_edge_mavlink_driver.vehicle import Refused   # noqa: E402

CONNECTION = "udpin:0.0.0.0:14540"


def pump(link, vehicle, stop):
    """What the driver's tick loop does: read, and heartbeat as a GCS."""
    while not stop.is_set():
        link.pump_once(timeout=0.2)
        vehicle.tick()


class Bench:

    def __init__(self, vehicle):
        self.v = vehicle
        self.failures = []

    def call(self, what, fn, expect=None):
        """Run one verb, print what came back and any words from the FC."""
        t0 = time.time()
        print(f"-> {what}")
        try:
            answer = fn()
            got = "accepted" if answer is None else f"accepted {answer}"
        except Refused as exc:
            got = f"refused: {exc}"
        except Exception as exc:                      # noqa: BLE001
            got = f"raised {type(exc).__name__}: {exc}"
        time.sleep(0.3)
        for text in self.v.link.texts_since(t0):
            print(f"     [fc] {text}")
        print(f"   {got}")
        if expect is not None and expect not in got:
            self.failures.append(f"{what}: wanted {expect!r}, got {got!r}")
        return got

    def attitude(self, tag, settle=0.0):
        """Where the mount is, after any time it needs to get there."""
        if settle:
            time.sleep(settle)
        print(f"   gimbal attitude {tag}: {self.v.gimbal_attitude()}")

    def stick(self, pitch_dps, yaw_dps, seconds):
        """A gimbal stick held for a while, then released, as the driver does."""
        print(f"-> gimbal_rate({pitch_dps}, {yaw_dps}) at 10 Hz for {seconds}s")
        end = time.time() + seconds
        try:
            while time.time() < end:
                self.v.gimbal_rate(pitch_dps, yaw_dps)
                time.sleep(0.1)
            self.attitude("while held")
            self.v.gimbal_rate(0, 0)
        except Refused as exc:
            print(f"   refused: {exc}")
            return
        time.sleep(1.0)
        self.attitude("1 s after the zeros")


def gimbal_round(bench, where):
    print(f"\n--- gimbal, {where} ---")
    bench.attitude("at rest")
    bench.call("gimbal_point(-45, 0, absolute=True)",
               lambda: bench.v.gimbal_point(-45.0, 0.0, True), expect="accepted")
    bench.attitude("after the absolute -45", settle=1.5)
    bench.call("gimbal_point(+15, 0, absolute=False)",
               lambda: bench.v.gimbal_point(15.0, 0.0, False), expect="accepted")
    bench.attitude("after the relative +15, wanted about -30", settle=1.5)
    bench.call("gimbal_point(-15, 90, absolute=True)",
               lambda: bench.v.gimbal_point(-15.0, 90.0, True), expect="accepted")
    bench.attitude("after the earth angle", settle=1.5)
    bench.call("gimbal_point(-20, 0, absolute=False, duration_s=2.0)",
               lambda: bench.v.gimbal_point(-20.0, 0.0, False, duration_s=2.0),
               expect="accepted")
    bench.attitude("after the one with a duration", settle=1.5)
    bench.call("gimbal_point(0, 0, absolute=True)",
               lambda: bench.v.gimbal_point(0.0, 0.0, True), expect="accepted")
    bench.stick(-10.0, 0.0, 3.0)
    bench.stick(0.0, 20.0, 2.0)
    bench.call("gimbal_point(0, 0, absolute=True)",
               lambda: bench.v.gimbal_point(0.0, 0.0, True), expect="accepted")


def home_round(bench, where):
    print(f"\n--- home point, {where} ---")
    v = bench.v
    here = v.link.m.messages.get("GLOBAL_POSITION_INT")
    if here is None:
        print("   no GLOBAL_POSITION_INT yet, skipping")
        return
    lat, lon = here.lat / 1e7, here.lon / 1e7
    print(f"   aircraft at {lat:.7f} {lon:.7f} {v.amsl():.2f} m amsl")
    bench.call(f"set_home({lat + 0.0002:.7f}, {lon + 0.0002:.7f}, 500.0)",
               lambda: v.set_home(lat + 0.0002, lon + 0.0002, 500.0), expect="accepted")
    show_home(v)
    bench.call(f"set_home({lat:.7f}, {lon:.7f}, None)",
               lambda: v.set_home(lat, lon, None), expect="accepted")
    show_home(v)


def show_home(v):
    time.sleep(2.5)     # HOME_POSITION is a 0.5 Hz stream
    h = v.link.m.messages.get("HOME_POSITION")
    if h is None:
        print("   HOME_POSITION not seen")
        return
    print(f"   HOME_POSITION {h.latitude / 1e7:.7f} {h.longitude / 1e7:.7f} "
          f"{h.altitude / 1000.0:.2f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--air", action="store_true", help="take off and repeat up there")
    ap.add_argument("--connection", default=CONNECTION)
    args = ap.parse_args()

    link = MavlinkLink(args.connection)
    link.connect()
    vehicle = PX4(link)
    stop = threading.Event()
    reader = threading.Thread(target=pump, args=(link, vehicle, stop), daemon=True)
    reader.start()
    time.sleep(2.0)
    link.request_streams()
    time.sleep(1.0)

    bench = Bench(vehicle)
    print(f"mode {vehicle.mode_name()}  armed {vehicle.armed()}  "
          f"in_air {vehicle.in_air()}  alt {link.state['alt']:.2f}")

    try:
        gimbal_round(bench, "on the ground")
        home_round(bench, "on the ground")

        print("\n--- compass calibration, on the ground ---")
        bench.call("compass_calibration(start=True)",
                   lambda: vehicle.compass_calibration(True), expect="accepted")
        time.sleep(2.0)
        bench.call("compass_calibration(start=False)",
                   lambda: vehicle.compass_calibration(False), expect="accepted")

        print("\n--- refused with the motors running ---")
        # the calibration worker takes a moment to unwind after a cancel, and
        # until it has, PX4 answers an arm with "Arming denied: calibrating"
        for _ in range(6):
            ok, reason = vehicle.set_armed(True)
            if ok:
                break
            time.sleep(2.0)
        print(f"-> arm: {ok} {reason}")
        if ok:
            bench.call("compass_calibration(start=True) while armed",
                       lambda: vehicle.compass_calibration(True),
                       expect="motors running")
            ok, reason = vehicle.set_armed(False)
            print(f"-> disarm: {ok} {reason}")

        if args.air:
            print("\n--- takeoff ---")
            ok, reason = vehicle.takeoff(4.0)
            print(f"   takeoff {ok} {reason} at {link.state['alt']:.2f} m")
            if ok:
                gimbal_round(bench, "in the air")
                home_round(bench, "in the air")
                print("\n--- land ---")
                print("   land", vehicle.land())
                for _ in range(40):
                    time.sleep(1.0)
                    if not vehicle.in_air():
                        break
                print(f"   landed, armed {vehicle.armed()}, "
                      f"alt {link.state['alt']:.2f}")
                vehicle.set_armed(False)
    finally:
        stop.set()
        reader.join(timeout=2.0)
        link.close()

    print("\n=== failures ===" if bench.failures else "\n=== all steps did what was expected ===")
    for line in bench.failures:
        print(" ", line)
    return 1 if bench.failures else 0


if __name__ == "__main__":
    sys.exit(main())
