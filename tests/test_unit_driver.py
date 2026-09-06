"""Unit tests for the driver: no aircraft, no broker, no network.

They cover arm/disarm and its status reply. The link itself is covered in
test_unit_link.py.
"""

import sys
import threading
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cyberwave_edge_mavlink_driver.hardware import MavlinkVehicle  # noqa: E402,F401


# --- command parsing and reply, at the driver layer -------------------

class FakeMQ:
    connected = True

    def __init__(self):
        self.replies = []
        self.published = []

    def publish_command_message(self, twin_uuid, payload):
        self.replies.append(payload)

    def publish(self, topic, payload):
        self.published.append((topic, payload))

    def subscribe_command_message(self, twin_uuid, cb):
        pass

    def connect(self):
        pass


class FakeVehicle:
    def __init__(self):
        self.state = {"armed": False, "mode_name": "STABILIZE",
                      "servo_pwm": (1000, 1000, 1000, 1000)}
        self.calls = []
        self.result = (True, "")

    def set_armed(self, arm, force=False, timeout=5.0):
        self.calls.append(("set_armed", arm, force))
        self.state["armed"] = arm and self.result[0]
        return self.result

    def send_velocity_body(self, *a):
        self.calls.append(("velocity", a))


@pytest.fixture
def driver(monkeypatch):
    from cyberwave_edge_mavlink_driver import driver as drv
    monkeypatch.setattr(drv, "Cyberwave", lambda *a, **k: types.SimpleNamespace(mqtt=FakeMQ()))
    monkeypatch.setattr(drv, "MavlinkVehicle", lambda conn: FakeVehicle())
    return drv.CyberwaveEdgeMavlinkDriver(twin_uuid="twin-uuid", api_key="x")


def run_one_command(d, env):
    """Push one envelope through the real MQTT callback + command worker."""
    d._on_command(env)
    t = threading.Thread(target=d._command_loop, daemon=True)
    t.start()
    deadline = time.time() + 3
    while time.time() < deadline and not d._mq.replies:
        time.sleep(0.02)
    d._stop.set()
    t.join(timeout=2)
    d._stop.clear()
    return d._mq.replies[-1] if d._mq.replies else None


def test_arm_command_parsed_and_answered(driver):
    reply = run_one_command(driver, {"source_type": "tele", "command": "arm",
                                     "data": {}})
    assert reply["status"] == "ok"
    assert reply["ok"] is True
    assert reply["command"] == "arm"
    assert reply["armed"] is True
    assert reply["reason"] == ""
    assert reply["motors_pwm"] == [1000, 1000, 1000, 1000]


@pytest.mark.parametrize("cmd, data, expected", [
    ("arm", {}, ("set_armed", True, False)),
    ("arm", {"force": True}, ("set_armed", True, True)),
    ("disarm", {}, ("set_armed", False, False)),
    ("disarm", {"force": True}, ("set_armed", False, True)),
])
def test_arm_disarm_reach_the_vehicle(driver, cmd, data, expected):
    run_one_command(driver, {"source_type": "tele", "command": cmd, "data": data})
    assert driver.vehicle.calls[0] == expected


def test_refused_arm_reply_carries_the_reason(driver):
    driver.vehicle.result = (False, "Arm: RC not found")
    reply = run_one_command(driver, {"source_type": "tele", "command": "arm",
                                     "data": {}})
    assert reply["status"] == "error"
    assert reply["ok"] is False
    assert reply["armed"] is False
    assert reply["reason"] == "Arm: RC not found"


def test_arm_publishes_vehicle_state_to_the_telemetry_topic(driver):
    run_one_command(driver, {"source_type": "tele", "command": "arm", "data": {}})
    topic, payload = driver._mq.published[-1]
    assert topic == "cyberwave/twin/twin-uuid/telemetry"
    assert payload["type"] == "vehicle_state"
    assert payload["armed"] is True
    assert payload["mode"] == "STABILIZE"


def test_sim_tele_arm_is_dropped(driver):
    driver._on_command({"source_type": "sim_tele", "command": "arm", "data": {}})
    assert driver._cmd_queue.empty()


def test_our_own_reply_is_not_re_executed(driver):
    driver._on_command({"status": "ok", "ok": True, "command": "arm",
                        "source_type": "tele"})
    assert driver._cmd_queue.empty()
