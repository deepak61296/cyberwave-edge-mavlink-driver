"""The driver: command parsing, replies, the source filter, sticks and the
telemetry payloads. No aircraft, no broker, no network."""

import asyncio
import sys
import threading
import time
import types
from pathlib import Path

import pytest
from cyberwave.driver import DriverOperationMode

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cyberwave_edge_mavlink_driver import contract  # noqa: E402
from cyberwave_edge_mavlink_driver.driver import MavlinkDriver  # noqa: E402
from cyberwave_edge_mavlink_driver.telemetry import PROP_JOINTS, PropSpin  # noqa: E402
from cyberwave_edge_mavlink_driver.vehicle import Vehicle  # noqa: E402


class FakeMQ:
    connected = True

    def __init__(self):
        self.replies = []

    def publish_command_message(self, twin_uuid, payload):
        self.replies.append(payload)

    def connect(self):
        pass


class FakeVehicle(Vehicle):
    """Records the verbs; armed and in_air are the real ones."""

    name = "fake"

    def __init__(self, link):
        super().__init__(link)
        self.calls = []
        self.result = (True, "")
        self.is_returning = False

    def mode_name(self):
        return "STABILIZE"

    def returning(self):
        return self.is_returning

    def set_armed(self, arm, force=False, timeout=5.0):
        self.calls.append(("set_armed", arm, force))
        self.link.state["armed"] = arm and self.result[0]
        return self.result

    def kill(self):
        self.calls.append(("kill",))
        return self.result

    def takeoff(self, altitude):
        self.calls.append(("takeoff", altitude))
        return self.result

    def land(self):
        self.calls.append(("land",))
        return self.result

    def return_to_home(self):
        self.calls.append(("return_to_home",))
        return self.result

    def hold(self):
        self.calls.append(("hold",))
        return self.result

    def set_home_here(self):
        self.calls.append(("set_home_here",))
        return self.result

    def reboot(self):
        self.calls.append(("reboot",))
        return self.result

    def prepare_sticks(self):
        self.calls.append(("prepare",))

    def release_sticks(self):
        self.calls.append(("release",))

    def send_velocity_body(self, *a):
        self.calls.append(("velocity", a))


@pytest.fixture
def driver():
    mq = FakeMQ()
    twin = types.SimpleNamespace(uuid="twin-uuid", client=types.SimpleNamespace(mqtt=mq))
    d = MavlinkDriver(twin=twin)
    d.vehicle = d.telemetry.vehicle = FakeVehicle(d.link)
    d.stream_requests = []
    d.link.m = types.SimpleNamespace(
        target_system=1, target_component=1,
        mav=types.SimpleNamespace(
            request_data_stream_send=lambda *a: d.stream_requests.append(a),
            command_long_send=lambda *a: None))
    d.link.state["last_heartbeat"] = time.time()
    d.link.state["last_attitude"] = time.time()
    d.link.state["servo_pwm"] = (1000, 1000, 1000, 1000)
    return d


def until(cond, timeout=5.0):
    """Wait for work the driver handed to a thread."""
    end = time.time() + timeout
    while time.time() < end and not cond():
        time.sleep(0.01)
    assert cond()


def send(d, envelope):
    """One envelope through the async handler, the reply if any."""
    asyncio.run(d._on_command(envelope))
    return d.client.mqtt.replies[-1] if d.client.mqtt.replies else None


# --- discrete commands ------------------------------------------------

def test_arm_command_parsed_and_answered(driver):
    reply = send(driver, {"source_type": "tele", "command": "arm", "data": {}})
    assert reply["status"] == "ok"
    assert reply["ok"] is True
    assert reply["command"] == "arm"
    assert reply["armed"] is True
    assert reply["reason"] == ""
    assert reply["flight_state"] == "motors_on"


@pytest.mark.parametrize("cmd, data, expected", [
    ("arm", {}, ("set_armed", True, False)),
    ("arm", {"force": True}, ("set_armed", True, True)),
    ("disarm", {}, ("set_armed", False, False)),
    ("disarm", {"force": True}, ("set_armed", False, True)),
    ("kill", {}, ("kill",)),
    ("emergency_stop", {}, ("hold",)),
    ("brake", {}, ("hold",)),
    ("cancel_landing", {}, ("hold",)),
    ("set_home_here", {}, ("set_home_here",)),
    ("reboot", {}, ("reboot",)),
])
def test_commands_reach_the_vehicle(driver, cmd, data, expected):
    send(driver, {"source_type": "tele", "command": cmd, "data": data})
    assert driver.vehicle.calls[-1] == expected


def test_takeoff_in_the_air_is_refused_before_the_backend_runs(driver):
    driver.link.state["armed"] = True
    driver.link.state["alt"] = 3.0
    reply = send(driver, {"source_type": "tele", "command": "takeoff", "data": {}})
    assert reply["status"] == "error"
    assert reply["reason"] == "already in air"
    assert driver.vehicle.calls == []


def test_kill_in_the_air_needs_force(driver):
    driver.link.state["armed"] = True
    driver.link.state["alt"] = 3.0
    reply = send(driver, {"source_type": "tele", "command": "kill", "data": {}})
    assert reply["status"] == "error"
    assert reply["reason"] == "in air, send force to override"
    assert driver.vehicle.calls == []


def test_disarm_in_the_air_needs_force(driver):
    driver.link.state["armed"] = True
    driver.link.state["alt"] = 3.0
    reply = send(driver, {"source_type": "tele", "command": "disarm", "data": {}})
    assert reply["status"] == "error"
    assert reply["reason"] == "in air, send force to override"
    assert driver.vehicle.calls == []
    reply = send(driver, {"source_type": "tele", "command": "disarm", "data": {"force": True}})
    assert reply["status"] == "ok"
    assert driver.vehicle.calls == [("set_armed", False, True)]


def test_kill_in_the_air_with_force_cuts_the_motors(driver):
    driver.link.state["armed"] = True
    driver.link.state["alt"] = 3.0
    reply = send(driver, {"source_type": "tele", "command": "kill", "data": {"force": True}})
    assert reply["status"] == "ok"
    assert driver.vehicle.calls == [("kill",)]


def test_kill_on_the_ground_needs_no_force(driver):
    driver.link.state["armed"] = True
    reply = send(driver, {"source_type": "tele", "command": "kill", "data": {}})
    assert reply["status"] == "ok"
    assert driver.vehicle.calls == [("kill",)]


def test_emergency_stop_hovers_and_leaves_the_motors_alone(driver):
    driver.link.state["armed"] = True
    driver.link.state["alt"] = 3.0
    reply = send(driver, {"source_type": "tele", "command": "emergency_stop", "data": {}})
    assert reply["status"] == "ok"
    assert driver.vehicle.calls == [("hold",)]
    assert driver.link.state["armed"] is True


def test_refused_arm_reply_carries_the_reason(driver):
    driver.vehicle.result = (False, "Arm: RC not found")
    reply = send(driver, {"source_type": "tele", "command": "arm", "data": {}})
    assert reply["status"] == "error"
    assert reply["ok"] is False
    assert reply["armed"] is False
    assert reply["reason"] == "Arm: RC not found"


def test_every_verb_is_refused_without_a_heartbeat(driver):
    driver.link.state["last_heartbeat"] = 0.0
    for cmd in ("arm", "takeoff", "kill", "brake"):
        reply = send(driver, {"source_type": "tele", "command": cmd, "data": {}})
        assert reply["status"] == "error"
        assert reply["reason"] == "not connected"
    assert driver.vehicle.calls == []
    reply = send(driver, {"source_type": "tele", "command": "stop", "data": {}})
    assert reply["status"] == "ok"              # the contract's one exception


def test_unknown_command_is_answered_not_dropped(driver):
    reply = send(driver, {"source_type": "tele", "command": "calibrate_compass", "data": {}})
    assert reply["status"] == "error"
    assert "not implemented" in reply["reason"]


def test_sim_tele_arm_is_dropped(driver):
    assert send(driver, {"source_type": "sim_tele", "command": "arm", "data": {}}) is None
    assert driver.vehicle.calls == []


def test_sim_tele_arm_runs_when_enabled(driver):
    driver.accept_sim_tele = True
    assert send(driver, {"source_type": "sim_tele", "command": "arm", "data": {}})["ok"]


@pytest.mark.parametrize("source", ["edit", "edge", None])
def test_other_sources_are_dropped(driver, source):
    assert send(driver, {"source_type": source, "command": "arm", "data": {}}) is None


def test_our_own_reply_is_not_re_executed(driver):
    send(driver, {"status": "ok", "ok": True, "command": "arm", "source_type": "tele"})
    assert driver.vehicle.calls == []


def test_discrete_command_releases_the_sticks_first(driver):
    driver._stick, driver._stick_at, driver._sticks_live = (1, 0, 0, 0), time.time(), True
    send(driver, {"source_type": "tele", "command": "land", "data": {}})
    assert driver._stick is None
    assert driver.vehicle.calls[0] == ("release",)


@pytest.mark.parametrize("urgent", contract.URGENT)
def test_an_urgent_verb_cuts_a_running_one_short(driver, urgent):
    """A kill must not queue behind a takeoff that waits half a minute."""
    v, in_takeoff = driver.vehicle, threading.Event()

    def takeoff(altitude):
        in_takeoff.set()
        v._wait(lambda: False, 30.0)
        return False, "gave up"
    v.takeoff = takeoff

    t = threading.Thread(target=send, args=(
        driver, {"source_type": "tele", "command": "takeoff", "data": {}}))
    t.start()
    assert in_takeoff.wait(5.0)
    t0 = time.time()
    reply = send(driver, {"source_type": "tele", "command": urgent, "data": {}})
    assert time.time() - t0 < 1.0
    assert reply["command"] == urgent
    assert reply["status"] == "ok"
    t.join(5.0)
    by_verb = {r["command"]: r for r in driver.client.mqtt.replies}
    assert by_verb["takeoff"]["reason"] == "superseded"


def test_stop_releases_the_sticks_and_replies_leaving_the_base_alone(driver):
    """The base would drop to NO_OP and rewire every subscription per burst."""
    driver._operation_mode = DriverOperationMode.TELEOP_REMOTE
    driver._stick, driver._stick_at, driver._sticks_live = (1, 0, 0, 0), time.time(), True
    asyncio.run(driver._on_stop_cmd({"source_type": "tele", "command": "stop", "data": {}}))
    reply = driver.client.mqtt.replies[-1]
    assert reply["command"] == "stop"
    assert reply["status"] == "ok"
    assert driver.vehicle.calls == [("release",)]
    assert driver._stick is None
    assert driver._operation_mode is DriverOperationMode.TELEOP_REMOTE


# --- sticks -------------------------------------------------------------

def test_stick_vector_from_a_burst(driver):
    driver._on_stick({"source_type": "tele", "command": "move_forward",
                      "data": {"linear_x": 0.7}})
    assert driver._stick == (0.7, 0, 0, 0)
    driver._on_stick({"source_type": "tele", "command": "turn_left",
                      "data": {"angular_z": 0.3}})
    assert driver._stick == (0, 0, 0, -0.3)
    driver._on_stick({"source_type": "tele", "command": "ascend", "data": {}})
    assert driver._stick == (0, 0, -contract.DEFAULT_SPEED, 0)


def test_sim_tele_stick_is_dropped(driver):
    driver._on_stick({"source_type": "sim_tele", "command": "move_forward", "data": {}})
    assert driver._stick is None


def test_fresh_stick_is_streamed_after_preparing_once(driver):
    driver._on_stick({"source_type": "tele", "command": "move_forward", "data": {}})
    asyncio.run(driver.on_tick())
    asyncio.run(driver.on_tick())
    until(lambda: ("prepare",) in driver.vehicle.calls)
    assert driver.vehicle.calls.count(("prepare",)) == 1
    assert [c for c in driver.vehicle.calls if c[0] == "velocity"] == \
        [("velocity", (1.0, 0, 0, 0))] * 2


def test_the_tick_keeps_streaming_through_the_mode_change(driver):
    in_prepare, finish = threading.Event(), threading.Event()

    def prepare():
        in_prepare.set()
        finish.wait(5.0)

    driver.vehicle.prepare_sticks = prepare
    driver._on_stick({"source_type": "tele", "command": "move_forward", "data": {}})
    asyncio.run(driver.on_tick())
    assert in_prepare.wait(5.0)
    asyncio.run(driver.on_tick())   # would block if prepare ran on the tick
    finish.set()
    assert driver.vehicle.calls.count(("velocity", (1.0, 0, 0, 0))) == 2


def test_sticks_are_dropped_while_a_verb_runs(driver):
    """A burst arriving mid-verb would re-engage GUIDED or OFFBOARD under it."""
    v, in_land, finish = driver.vehicle, threading.Event(), threading.Event()

    def land():
        in_land.set()
        finish.wait(5.0)
        return True, ""
    v.land = land

    t = threading.Thread(target=send, args=(
        driver, {"source_type": "tele", "command": "land", "data": {}}))
    t.start()
    assert in_land.wait(5.0)
    driver._on_stick({"source_type": "tele", "command": "move_forward", "data": {}})
    assert driver._stick is None
    driver._stick, driver._stick_at = (1.0, 0, 0, 0), time.time()   # slipped in before
    asyncio.run(driver.on_tick())
    assert ("velocity", (1.0, 0, 0, 0)) not in v.calls
    finish.set()
    t.join(5.0)
    driver._on_stick({"source_type": "tele", "command": "move_forward", "data": {}})
    assert driver._stick == (1.0, 0, 0, 0)      # taken again once the verb is done


def test_a_verb_during_prepare_waits_for_it(driver):
    """Two threads changing modes at once fight over the aircraft."""
    in_prepare, finish = threading.Event(), threading.Event()

    def prepare():
        in_prepare.set()
        finish.wait(5.0)
        driver.vehicle.calls.append(("prepare",))
    driver.vehicle.prepare_sticks = prepare

    driver._on_stick({"source_type": "tele", "command": "move_forward", "data": {}})
    asyncio.run(driver.on_tick())
    assert in_prepare.wait(5.0)
    t = threading.Thread(target=send, args=(
        driver, {"source_type": "tele", "command": "land", "data": {}}))
    t.start()
    time.sleep(0.2)
    assert ("release",) not in driver.vehicle.calls    # land waits for prepare to end
    finish.set()
    t.join(5.0)
    calls = driver.vehicle.calls
    assert calls.index(("prepare",)) < calls.index(("release",)) < calls.index(("land",))


def test_stick_expiry_releases_once(driver):
    driver._on_stick({"source_type": "tele", "command": "move_forward", "data": {}})
    asyncio.run(driver.on_tick())
    driver._stick_at -= contract.STICK_TIMEOUT_S + 0.1
    asyncio.run(driver.on_tick())
    asyncio.run(driver.on_tick())
    until(lambda: ("release",) in driver.vehicle.calls)
    assert driver.vehicle.calls.count(("release",)) == 1
    assert driver.vehicle.calls[-1] == ("release",)


def test_quiet_attitude_asks_for_streams_again_but_not_every_tick(driver):
    asyncio.run(driver.on_tick())
    assert driver.stream_requests == []
    driver.link.state["last_attitude"] = time.time() - 10
    asyncio.run(driver.on_tick())
    asyncio.run(driver.on_tick())
    assert len(driver.stream_requests) == 1


def test_lost_broker_flags_the_reconnect_loop(driver):
    asyncio.run(driver.on_tick())
    assert not driver._connection_lost.is_set()
    driver.client.mqtt.connected = False
    asyncio.run(driver.on_tick())
    assert driver._connection_lost.is_set()


# --- telemetry ------------------------------------------------------------

def test_vehicle_state_payload_shape(driver):
    payload = driver.telemetry.vehicle_state()
    assert payload["type"] == "vehicle_state"
    assert payload["armed"] is False
    assert payload["mode"] == "STABILIZE"
    assert payload["flight_state"] == "ready"
    assert payload["motors_pwm"] == [1000, 1000, 1000, 1000]
    assert payload["source_type"] == "edge"
    assert driver.telemetry.vehicle_state() is None               # unchanged, too soon
    driver.link.state["armed"] = True
    assert driver.telemetry.vehicle_state()["armed"] is True      # changed, goes out now


def test_position_and_rotation_payloads(driver):
    assert driver.telemetry.position() is None
    driver.link.state["ned"] = (1.0, 2.0, -3.0)
    assert driver.telemetry.position()["position"] == {"x": 2.0, "y": 1.0, "z": 3.0}
    assert driver.telemetry.rotation() is None
    driver.link.state["attitude"] = (0.0, 0.0, 0.0)
    assert set(driver.telemetry.rotation()["rotation"]) == {"w", "x", "y", "z"}


def test_props_spin_from_pwm(driver):
    assert PropSpin().payload(None) is None
    driver.link.state["servo_pwm"] = (1500, 1000, 65535, 1500)
    payload = driver.telemetry.prop_joints()
    assert set(payload["positions"]) == set(PROP_JOINTS)
    v = payload["velocities"]
    assert v["prop_1_joint"] == 30.0                 # half throttle, CCW
    assert v["prop_2_joint"] == 0.0                  # idle pwm is below the bound
    assert v["prop_3_joint"] == 0.0                  # 65535 is "unknown", not full
    assert v["prop_4_joint"] == -30.0                # CW


@pytest.mark.parametrize("connected, armed, in_air, returning, was_airborne, expected", [
    (False, True, True, False, True, "disconnected"),
    (True, False, False, False, False, "ready"),
    (True, True, False, False, False, "motors_on"),
    (True, True, True, False, True, "in_air"),
    (True, True, True, True, True, "returning"),
    (True, True, False, False, True, "landed"),
    (True, False, False, False, True, "ready"),
])
def test_flight_state(connected, armed, in_air, returning, was_airborne, expected):
    assert contract.flight_state(connected, armed, in_air, returning, was_airborne) == expected


def test_flight_state_follows_a_whole_flight(driver):
    state, link = driver.telemetry.flight_state, driver.link
    assert state() == "ready"
    link.state["armed"] = True
    assert state() == "motors_on"
    link.state["alt"] = 3.0
    assert state() == "in_air"
    driver.vehicle.is_returning = True
    assert state() == "returning"
    driver.vehicle.is_returning = False
    link.state["alt"] = 0.0
    assert state() == "landed"
    link.state["armed"] = False
    assert state() == "ready"           # motors off, that flight is over
    link.state["last_heartbeat"] = 0.0
    assert state() == "disconnected"


# --- the manifest ---------------------------------------------------------

def test_manifest_lists_every_verb_offline():
    manifest = MavlinkDriver.get_manifest(compiled=False)
    supported = manifest["mqtt"]["commands"]["supported"]
    names = [c["name"] if isinstance(c, dict) else c for c in supported]
    for verb in ("arm", "disarm", "brake", "kill", "takeoff", "stop"):
        assert verb in names
    for verb in contract.CONTINUOUS:
        entry = supported[names.index(verb)]
        assert entry["continuous"] is True
    assert set(manifest["mqtt"]["twin"]) == {"command", "telemetry", "position", "rotation"}
    assert set(manifest["mqtt"]["joint"]) == {"update"}
