"""The driver: command parsing, replies, the source filter, sticks and the
telemetry payloads. No aircraft, no broker, no network."""

import asyncio
import sys
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cyberwave_edge_mavlink_driver import contract  # noqa: E402
from cyberwave_edge_mavlink_driver.driver import MavlinkDriver  # noqa: E402
from cyberwave_edge_mavlink_driver.telemetry import PROP_JOINTS, PropSpin  # noqa: E402


class FakeMQ:
    connected = True

    def __init__(self):
        self.replies = []

    def publish_command_message(self, twin_uuid, payload):
        self.replies.append(payload)

    def connect(self):
        pass


class FakeVehicle:
    name = "fake"

    def __init__(self, link):
        self.link = link
        self.calls = []
        self.result = (True, "")

    def armed(self):
        return bool(self.link.state["armed"])

    def mode_name(self):
        return "STABILIZE"

    def in_air(self):
        return self.link.state["alt"] > 0.5

    def returning(self):
        return False

    def set_armed(self, arm, force=False, timeout=5.0):
        self.calls.append(("set_armed", arm, force))
        self.link.state["armed"] = arm and self.result[0]
        return self.result

    def kill(self):
        self.calls.append(("kill",))
        return self.result

    def hold(self):
        self.calls.append(("hold",))
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
    ("emergency_stop", {}, ("kill",)),
    ("brake", {}, ("hold",)),
    ("cancel_landing", {}, ("hold",)),
])
def test_commands_reach_the_vehicle(driver, cmd, data, expected):
    send(driver, {"source_type": "tele", "command": cmd, "data": data})
    assert driver.vehicle.calls[-1] == expected


def test_refused_arm_reply_carries_the_reason(driver):
    driver.vehicle.result = (False, "Arm: RC not found")
    reply = send(driver, {"source_type": "tele", "command": "arm", "data": {}})
    assert reply["status"] == "error"
    assert reply["ok"] is False
    assert reply["armed"] is False
    assert reply["reason"] == "Arm: RC not found"


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
    assert driver.vehicle.calls == [("prepare",), ("velocity", (1.0, 0, 0, 0)),
                                    ("velocity", (1.0, 0, 0, 0))]


def test_stick_expiry_releases_once(driver):
    driver._on_stick({"source_type": "tele", "command": "move_forward", "data": {}})
    asyncio.run(driver.on_tick())
    driver._stick_at -= contract.STICK_TIMEOUT_S + 0.1
    asyncio.run(driver.on_tick())
    asyncio.run(driver.on_tick())
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
    (True, False, False, False, True, "landed"),
])
def test_flight_state(connected, armed, in_air, returning, was_airborne, expected):
    assert contract.flight_state(connected, armed, in_air, returning, was_airborne) == expected


def test_flight_state_from_the_link(driver):
    assert driver.telemetry.flight_state() == "ready"
    driver.link.state["armed"] = True
    driver.link.state["alt"] = 3.0
    assert driver.telemetry.flight_state() == "in_air"
    driver.link.state["armed"] = False
    driver.link.state["alt"] = 0.0
    assert driver.telemetry.flight_state() == "landed"
    driver.link.state["last_heartbeat"] = 0.0
    assert driver.telemetry.flight_state() == "disconnected"


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
