"""The simulated aircraft: the model, the two profiles and the driver's
command path against sim://dji. No aircraft, no broker, no network."""

import math
import sys
import threading
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cyberwave_edge_mavlink_driver import simulated  # noqa: E402
from cyberwave_edge_mavlink_driver.driver import MavlinkDriver  # noqa: E402
from cyberwave_edge_mavlink_driver.simulated import (  # noqa: E402
    NOT_SUPPORTED, SimLink, SimVehicle,
)
from cyberwave_edge_mavlink_driver.telemetry import Telemetry  # noqa: E402
from cyberwave_edge_mavlink_driver.vehicle import pick_vehicle  # noqa: E402


class Clock:
    """Model time, moved by the test, so a climb costs no real seconds."""

    def __init__(self, t=1000.0):
        self.t = t

    def __call__(self):
        return self.t


def sim(connection="sim://quad"):
    link = SimLink(connection)
    link.connect()
    clock = Clock()
    return SimVehicle(link, now=clock), clock


def step(vehicle, clock, seconds, dt=0.05):
    """Fly the model forward, one tick at a time."""
    for _ in range(int(seconds / dt)):
        clock.t += dt
        vehicle.tick()


def ticking(vehicle, clock):
    """Step the model from a thread, for the verbs that wait for it."""
    stop = threading.Event()

    def loop():
        while not stop.wait(0.001):
            clock.t += 0.05
            vehicle.tick()

    threading.Thread(target=loop, daemon=True).start()
    return stop


def flying(vehicle, clock, altitude=3.0):
    """Take off and come back with the aircraft in the air."""
    stop = ticking(vehicle, clock)
    try:
        return vehicle.takeoff(altitude)
    finally:
        stop.set()


# --- the model --------------------------------------------------------

def test_takeoff_climbs_to_the_altitude_asked_for():
    v, clock = sim()
    assert flying(v, clock, 3.0) == (True, "")
    assert v.alt() == pytest.approx(3.0)
    assert v.armed() and v.in_air()
    assert v.takeoff_altitude(3.0) == 3.0
    assert v.mode_name() == "GUIDED"


def test_takeoff_refuses_once_the_aircraft_is_up():
    v, clock = sim()
    flying(v, clock, 2.0)
    assert v.takeoff(2.0) == (False, "already in air")


def test_land_reaches_the_ground_and_stops_the_motors():
    v, clock = sim()
    flying(v, clock, 3.0)
    assert v.land() == (True, "")
    step(v, clock, 4.0)
    assert v.alt() == 0.0
    assert not v.armed() and not v.in_air()
    assert v.mode_name() == "STABILIZE"


def test_sticks_move_the_position_in_the_body_frame():
    v, clock = sim()
    flying(v, clock, 3.0)
    v.prepare_sticks()
    assert v.task == "sticks"
    v.yaw = math.pi / 2                 # nose east, so forward is east
    v.send_velocity_body(1.0, 0.0, 0.0, 0.0)
    step(v, clock, 0.4)
    assert v.ned[0] == pytest.approx(0.0, abs=1e-6)
    assert v.ned[1] == pytest.approx(0.4, abs=1e-6)
    assert v.alt() == pytest.approx(3.0)


def test_a_stick_that_stops_refreshing_stops_the_aircraft():
    v, clock = sim()
    flying(v, clock, 3.0)
    v.prepare_sticks()
    v.send_velocity_body(2.0, 0.0, 0.0, 0.0)
    step(v, clock, 2.0)                 # only the first half second counts
    assert v.ned[0] == pytest.approx(1.0, abs=1e-6)
    assert v.stick == simulated.ZERO


def test_climbing_and_turning_on_the_sticks():
    v, clock = sim()
    flying(v, clock, 3.0)
    v.prepare_sticks()
    for _ in range(10):                 # a burst, refreshed the way the driver does
        v.send_velocity_body(0.0, 0.0, -1.0, 1.0)
        step(v, clock, 0.1)
    assert v.alt() == pytest.approx(4.0)
    assert v.yaw == pytest.approx(1.0)


def test_release_sticks_leaves_the_aircraft_hovering():
    v, clock = sim()
    flying(v, clock, 3.0)
    v.prepare_sticks()
    v.send_velocity_body(1.0, 0.0, 0.0, 0.0)
    step(v, clock, 0.2)
    v.release_sticks()
    step(v, clock, 1.0)
    assert v.ned[0] == pytest.approx(0.2, abs=1e-6)
    assert v.task is None and v.in_air()


def test_return_to_home_flies_back_and_lands():
    v, clock = sim()
    flying(v, clock, 3.0)
    v.ned[0], v.ned[1] = 10.0, 5.0
    assert v.return_to_home() == (True, "")
    assert v.returning() and v.mode_name() == "RTL"
    step(v, clock, 10.0)
    assert v.ned[0] == pytest.approx(0.0, abs=0.1)
    assert v.ned[1] == pytest.approx(0.0, abs=0.1)
    assert v.alt() == 0.0 and not v.armed()


def test_set_home_here_is_where_a_return_ends_up():
    v, clock = sim()
    flying(v, clock, 3.0)
    v.ned[0], v.ned[1] = 8.0, 0.0
    v.set_home_here()
    v.ned[0] = 20.0
    v.return_to_home()
    step(v, clock, 10.0)
    assert v.ned[0] == pytest.approx(8.0, abs=0.1)


def test_hold_cancels_the_task_and_zeroes_the_sticks():
    v, clock = sim()
    flying(v, clock, 3.0)
    v.land()
    assert v.hold() == (True, "")
    step(v, clock, 2.0)
    assert v.task is None and v.alt() == pytest.approx(3.0)


def test_kill_in_the_air_drops_the_aircraft():
    v, clock = sim()
    flying(v, clock, 3.0)
    assert v.kill() == (True, "")
    assert not v.armed()
    step(v, clock, 1.0)
    assert v.alt() == 0.0


def test_reboot_and_compass_calibration_refuse_with_the_motors_running():
    v, clock = sim()
    assert v.compass_calibration(True) == (True, "")
    assert v.calibrating
    assert v.set_armed(True) == (True, "")
    assert v.reboot() == (False, "motors running")
    assert v.compass_calibration(True) == (False, "motors running")
    assert v.compass_calibration(False) == (True, "")    # stopping is never refused
    assert v.set_armed(False) == (True, "")
    assert v.reboot() == (True, "")


def test_set_home_takes_coordinates_and_checks_them():
    v, _ = sim()
    assert v.set_home(12.9716, 77.5946, 900.0) == (True, "")
    assert v.home_fix == (12.9716, 77.5946, 900.0)
    assert v.set_home(120.0, 0.0, 0.0) == (False, "coordinates out of range")


def test_an_unknown_profile_flies_the_quad():
    v, _ = sim("sim://something")
    assert v.profile is simulated.QUAD
    assert v.name == "sim-quad"


# --- the DJI profile --------------------------------------------------

def test_dji_has_no_arm_disarm_or_kill():
    v, _ = sim("sim://dji")
    assert v.set_armed(True) == (False, NOT_SUPPORTED)
    assert v.set_armed(False) == (False, NOT_SUPPORTED)
    assert v.kill() == (False, NOT_SUPPORTED)
    assert not v.armed()


def test_dji_takes_off_to_its_own_height():
    v, clock = sim("sim://dji")
    assert flying(v, clock, 5.0) == (True, "")
    assert v.alt() == pytest.approx(1.2)
    assert v.takeoff_altitude(5.0) == 1.2
    assert v.mode_name() == "HOVER"


def test_dji_land_parks_low_until_a_second_land_confirms():
    v, clock = sim("sim://dji")
    flying(v, clock, 1.2)
    assert v.land() == (True, "", {"pending_confirmation": True})
    step(v, clock, 5.0)
    assert v.alt() == pytest.approx(0.7)
    assert v.armed() and v.mode_name() == "AUTO_LANDING"
    assert v.land() == (True, "")
    step(v, clock, 2.0)
    assert v.alt() == 0.0 and not v.armed()


def test_dji_return_to_home_waits_for_the_confirm():
    v, clock = sim("sim://dji")
    flying(v, clock, 1.2)
    v.ned[0] = 6.0
    assert v.return_to_home() == (True, "", {"pending_confirmation": True})
    step(v, clock, 2.0)
    assert not v.returning() and v.ned[0] == 6.0    # still parked on the prompt
    assert v.return_to_home() == (True, "")
    assert v.mode_name() == "GO_HOME"
    step(v, clock, 5.0)
    assert v.ned[0] == pytest.approx(0.0, abs=0.1) and not v.armed()


def test_cancel_clears_a_pending_confirm():
    v, clock = sim("sim://dji")
    flying(v, clock, 1.2)
    v.return_to_home()
    v.hold()
    assert v.pending is None
    v.return_to_home()                  # the next one asks again
    assert v.pending == "rtl"


# --- the gimbal -------------------------------------------------------

def test_gimbal_point_clamps_to_the_pitch_limit():
    v, clock = sim("sim://dji")
    assert v.gimbal_point(-120.0, None, True) == (True, "")
    step(v, clock, 1.0)
    assert v.gimbal_attitude() == (-90.0, 0.0)
    assert v.gimbal_point(200.0, None, True) == (True, "")
    step(v, clock, 1.0)
    assert v.gimbal_attitude() == (60.0, 0.0)


def test_gimbal_point_relative_and_over_a_duration():
    v, clock = sim()
    v.gimbal_point(-10.0, 20.0, True)
    step(v, clock, 1.0)
    v.gimbal_point(-10.0, None, False)
    step(v, clock, 1.0)
    assert v.gimbal_attitude() == (-20.0, 20.0)
    v.gimbal_point(-50.0, None, True, duration_s=2.0)
    step(v, clock, 1.0)
    assert v.gimbal_attitude()[0] == pytest.approx(-35.0)   # half way there
    step(v, clock, 1.5)
    assert v.gimbal_attitude()[0] == pytest.approx(-50.0)


def test_gimbal_yaw_is_refused_on_dji_and_limited_on_the_quad():
    v, clock = sim("sim://dji")
    assert v.gimbal_point(None, 30.0, True) == (False, NOT_SUPPORTED)
    assert v.gimbal_rate(0.0, 10.0) == (False, NOT_SUPPORTED)
    q, qclock = sim()
    assert q.gimbal_point(None, 400.0, True) == (True, "")
    step(q, qclock, 2.0)
    assert q.gimbal_attitude() == (0.0, 160.0)


def test_gimbal_rate_runs_until_it_stops_refreshing():
    v, clock = sim("sim://dji")
    assert v.gimbal_rate(-10.0, 0.0) == (True, "")
    step(v, clock, 0.4)
    assert v.gimbal_attitude()[0] == pytest.approx(-4.0)
    step(v, clock, 2.0)                 # no refresh, so it stopped at 0.5 s
    assert v.gimbal_attitude()[0] == pytest.approx(-5.0)


# --- telemetry --------------------------------------------------------

def test_vehicle_state_carries_the_gimbal_and_the_props_spin():
    v, clock = sim("sim://dji")
    telemetry = Telemetry(v.link)
    telemetry.vehicle = v
    payload = telemetry.vehicle_state()
    assert payload["flight_state"] == "ready"
    assert payload["mode"] == "READY"
    assert payload["motors_pwm"] == [1000, 1000, 1000, 1000]
    assert payload["gimbal_pitch"] == 0.0 and payload["gimbal_yaw"] == 0.0
    flying(v, clock, 1.2)
    payload = telemetry.vehicle_state()
    assert payload["flight_state"] == "in_air"
    assert payload["motors_pwm"] == [1100, 1100, 1100, 1100]
    assert telemetry.prop_joints()["velocities"]["prop_1_joint"] > 0
    assert telemetry.position()["position"]["z"] == pytest.approx(1.2)


def test_vehicle_state_has_no_gimbal_without_one():
    v, _ = sim()
    v.profile = v.profile._replace(gimbal=(None, None))
    telemetry = Telemetry(v.link)
    telemetry.vehicle = v
    assert v.gimbal_attitude() is None
    assert "gimbal_pitch" not in telemetry.vehicle_state()


# --- the driver's command path ----------------------------------------

class FakeMQ:
    connected = True

    def __init__(self):
        self.replies = []

    def publish_command_message(self, twin_uuid, payload):
        self.replies.append(payload)


@pytest.fixture
def dji(monkeypatch):
    """The driver, wired to sim://dji, with the model on a thread."""
    monkeypatch.setenv("MAVLINK_CONNECTION", "sim://dji")
    mq = FakeMQ()
    twin = types.SimpleNamespace(uuid="twin-uuid", client=types.SimpleNamespace(mqtt=mq))
    driver = MavlinkDriver(twin=twin)
    driver.link.connect()
    driver.vehicle = driver.telemetry.vehicle = pick_vehicle(driver.link)
    clock = Clock()
    driver.vehicle.now, driver.vehicle.at = clock, clock()
    stop = ticking(driver.vehicle, clock)
    driver.mq = mq
    yield driver
    stop.set()


def run(driver, command, **data):
    """One command through the driver, as the command topic would send it."""
    driver._run({"source_type": "tele", "command": command, "data": data,
                 "timestamp": time.time()})
    return driver.mq.replies[-1]


def test_driver_runs_the_discrete_verbs_against_the_dji_sim(dji):
    assert isinstance(dji.link, SimLink)
    assert dji.vehicle.name == "sim-dji"

    reply = run(dji, "arm")
    assert (reply["status"], reply["reason"]) == ("error", NOT_SUPPORTED)
    assert run(dji, "kill")["reason"] == NOT_SUPPORTED
    assert run(dji, "land")["reason"] == "not in air"
    assert run(dji, "brake")["reason"] == "not in air"
    assert run(dji, "set_home_here")["status"] == "ok"
    assert run(dji, "reboot")["status"] == "ok"

    reply = run(dji, "takeoff", altitude=4.0)
    assert reply["status"] == "ok" and reply["altitude_m"] == 1.2
    assert reply["flight_state"] == "in_air" and reply["mode"] == "HOVER"
    assert run(dji, "takeoff")["reason"] == "already in air"
    assert run(dji, "kill")["reason"] == "in air, send force to override"
    assert run(dji, "kill", force=True)["reason"] == NOT_SUPPORTED
    assert run(dji, "hover")["status"] == "ok"
    assert run(dji, "no_such_verb")["reason"] == NOT_SUPPORTED

    reply = run(dji, "land")
    assert reply["status"] == "ok" and reply["pending_confirmation"] is True
    assert run(dji, "cancel_landing")["status"] == "ok"
    run(dji, "land")
    assert run(dji, "land")["status"] == "ok"
    until(lambda: dji.telemetry.flight_state() == "ready")
    assert not dji.vehicle.armed()


def until(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end and not cond():
        time.sleep(0.01)
    assert cond()
