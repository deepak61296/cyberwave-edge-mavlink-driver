"""The simulated aircraft: the model, the two profiles and the driver's
command path against sim://dji. No aircraft, no broker, no network."""

import asyncio
import math
import sys
import threading
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cyberwave_edge_mavlink_driver import simulated  # noqa: E402
from cyberwave_edge_mavlink_driver.contract import NOT_SUPPORTED  # noqa: E402
from cyberwave_edge_mavlink_driver.driver import MavlinkDriver  # noqa: E402
from cyberwave_edge_mavlink_driver.simulated import SimLink, SimVehicle  # noqa: E402
from cyberwave_edge_mavlink_driver.telemetry import Telemetry  # noqa: E402
from cyberwave_edge_mavlink_driver.vehicle import NAN, Refused, pick_vehicle  # noqa: E402


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


def test_reboot_refuses_with_the_motors_running():
    v, _ = sim()
    v.compass_calibration(True)
    assert v.calibrating
    assert v.set_armed(True) == (True, "")
    assert v.reboot() == (False, "motors running")
    assert v.set_armed(False) == (True, "")
    assert v.reboot() == (True, "")


def test_set_home_records_the_coordinates_it_is_given():
    v, _ = sim()
    v.set_home(12.9716, 77.5946, 900.0)
    assert v.home_fix == (12.9716, 77.5946, 900.0)
    v.set_home(12.0, 77.0, None)            # no altitude keeps the one home has
    assert v.home_fix == (12.0, 77.0, 900.0)


def test_an_unknown_profile_flies_the_quad():
    v, _ = sim("sim://something")
    assert v.profile is simulated.QUAD
    assert v.name == "sim-quad"


# --- the DJI profile --------------------------------------------------

def test_dji_arming_is_implicit_and_the_motor_cut_has_no_key():
    v, clock = sim("sim://dji")
    assert v.set_armed(True) == (True, "", {"implicit": True})
    assert v.set_armed(False) == (True, "", {"implicit": True})
    assert not v.armed()                # ok, and nothing happened
    assert v.kill() == (False, NOT_SUPPORTED)
    flying(v, clock, 1.2)
    assert v.set_armed(False, force=True) == (False, NOT_SUPPORTED)
    assert v.armed()


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
    v.gimbal_point(-120.0, NAN, True)
    step(v, clock, 1.0)
    assert v.gimbal_attitude() == (-90.0, 0.0)
    v.gimbal_point(200.0, NAN, True)
    step(v, clock, 1.0)
    assert v.gimbal_attitude() == (60.0, 0.0)


def test_gimbal_point_relative_and_over_a_duration():
    v, clock = sim()
    v.gimbal_point(-10.0, 20.0, True)
    step(v, clock, 1.0)
    v.gimbal_point(-10.0, NAN, False)
    step(v, clock, 1.0)
    assert v.gimbal_attitude() == (-20.0, 20.0)
    v.gimbal_point(-50.0, NAN, True, duration_s=2.0)
    step(v, clock, 1.0)
    assert v.gimbal_attitude()[0] == pytest.approx(-35.0)   # half way there
    step(v, clock, 1.5)
    assert v.gimbal_attitude()[0] == pytest.approx(-50.0)


def test_gimbal_yaw_is_refused_on_dji_and_limited_on_the_quad():
    v, _ = sim("sim://dji")
    with pytest.raises(Refused, match=NOT_SUPPORTED):
        v.gimbal_point(NAN, 30.0, True)
    with pytest.raises(Refused, match=NOT_SUPPORTED):
        v.gimbal_rate(0.0, 10.0)
    v.gimbal_rate(-5.0, NAN)            # the axis left out is not a refusal
    q, qclock = sim()
    q.gimbal_point(NAN, 400.0, True)
    step(q, qclock, 2.0)
    assert q.gimbal_attitude() == (0.0, 160.0)


def test_gimbal_rate_runs_until_it_stops_refreshing():
    v, clock = sim("sim://dji")
    v.gimbal_rate(-10.0, NAN)
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
    assert (reply["status"], reply["implicit"], reply["armed"]) == ("ok", True, False)
    assert run(dji, "disarm")["implicit"] is True
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
    assert run(dji, "disarm", force=True)["reason"] == NOT_SUPPORTED
    assert run(dji, "hover")["status"] == "ok"
    assert run(dji, "no_such_verb")["reason"] == NOT_SUPPORTED

    reply = run(dji, "land")
    assert reply["status"] == "ok" and reply["pending_confirmation"] is True
    assert run(dji, "cancel_landing")["status"] == "ok"
    run(dji, "land")
    assert run(dji, "land")["status"] == "ok"
    until(lambda: dji.telemetry.flight_state() == "ready")
    assert not dji.vehicle.armed()


def test_driver_runs_the_dji_catalog_verbs_against_the_sim(dji):
    """The nine verbs the DJI catalog adds, on the aircraft's own terms."""
    assert run(dji, "reboot_aircraft")["status"] == "ok"

    assert run(dji, "set_gimbal_pitch", pitch=-40.0)["status"] == "ok"
    until(lambda: dji.vehicle.gimbal_attitude()[0] == -40.0)
    assert run(dji, "gimbal_rotate", pitch=-90.0, mode="absolute",
               duration=0.2)["status"] == "ok"
    until(lambda: dji.vehicle.gimbal_attitude()[0] == -90.0)
    assert dji.telemetry.vehicle_state()["gimbal_pitch"] == -90.0

    # the Mini 4 Pro gimbal does not yaw, so a yaw-only request is refused
    reply = run(dji, "gimbal_rotate", yaw=30.0)
    assert (reply["status"], reply["reason"]) == ("error", NOT_SUPPORTED)
    assert run(dji, "gimbal_rotate", roll=10.0)["reason"] == NOT_SUPPORTED

    assert run(dji, "gimbal_rotate_speed", pitch=300)["status"] == "ok"
    assert dji.vehicle.gimbal_dps == (30.0, 0.0)   # the SDK sends 0.1 deg/s

    assert run(dji, "set_home_location", latitude=12.9716,
               longitude=77.5946)["status"] == "ok"
    assert dji.vehicle.home_fix == (12.9716, 77.5946, None)
    assert run(dji, "set_home_location", latitude=200.0,
               longitude=0.0)["reason"] == "coordinates out of range"

    assert run(dji, "start_compass_calibration")["status"] == "ok"
    assert dji.vehicle.calibrating
    assert run(dji, "stop_compass_calibration")["status"] == "ok"
    assert not dji.vehicle.calibrating


def test_gimbal_sticks_move_the_camera_from_the_tick(dji):
    """gimbal_pitch_down is a stick, so the tick streams it and it expires."""
    dji._on_stick({"source_type": "tele", "command": "gimbal_pitch_down",
                   "data": {"rate": 20.0}, "timestamp": time.time()})
    dji._tick_gimbal(time.time())
    until(lambda: dji.vehicle.gimbal_attitude()[0] < -0.5)
    assert not dji.mq.replies                      # a stick draws no reply


def until(cond, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end and not cond():
        time.sleep(0.01)
    assert cond()


# --- the base's rules, on the simulated aircraft ----------------------

def test_a_cancelled_takeoff_is_superseded_and_keeps_its_motors(dji, monkeypatch):
    """cancel_takeoff is urgent now: it sets abort before it queues, the
    climb gives way, and the aircraft holds where it is with the motors on."""
    # the quad's profile in this harness: a DJI is at its 1.2 m before a
    # cancel could be typed, and the driver path is what is under test
    dji.vehicle.profile = simulated.QUAD

    started = threading.Event()

    def climb():
        started.set()
        run(dji, "takeoff", altitude=40.0)

    flier = threading.Thread(target=climb, daemon=True)
    flier.start()
    started.wait(1.0)
    time.sleep(0.05)

    asyncio.run(dji._on_command({"source_type": "tele", "command": "cancel_takeoff",
                                 "data": {}, "timestamp": time.time()}))
    flier.join(5.0)
    assert not flier.is_alive()

    said = {r["command"]: r for r in dji.mq.replies}
    assert said["takeoff"]["status"] == "error"
    assert said["takeoff"]["reason"] == "superseded"
    assert said["cancel_takeoff"]["status"] == "ok"
    assert dji.vehicle.armed() and dji.vehicle.in_air()
    assert 0.0 < dji.vehicle.alt() < 40.0
    assert dji.vehicle.task is None      # holding, not still climbing


@pytest.mark.parametrize("connection", ["sim://quad", "sim://dji"])
def test_a_takeoff_that_never_leaves_the_ground_stops_the_motors(connection, monkeypatch):
    """The base's takeoff_failed, which the backends gained with the review:
    no climb, no motors. The DJI has no arming key, so the model parks."""
    monkeypatch.setattr(simulated, "TAKEOFF_S", 0.05)
    v, _ = sim(connection)          # nothing ticks it, so it never climbs
    ok, reason = v.takeoff(3.0)
    assert not ok and reason.startswith("still climbing")
    assert not v.armed() and not v.in_air()
    assert v.task is None


def test_home_is_kept_above_mean_sea_level():
    """set_home takes AMSL after the review, and home_amsl reads it back
    without the HOME_POSITION message this link never carries."""
    v, _ = sim()
    assert v.home_amsl() is None
    v.set_home(12.9716, 77.5946, 921.0)
    assert v.home_fix == (12.9716, 77.5946, 921.0)
    assert v.home_amsl() == 921.0
    v.set_home(12.9716, 77.5946, None)      # no altitude keeps the height
    assert v.home_amsl() == 921.0
