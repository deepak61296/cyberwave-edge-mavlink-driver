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

from pymavlink import mavutil  # noqa: E402

from cyberwave_edge_mavlink_driver.hardware import MavlinkVehicle  # noqa: E402

class FakeLink:
    """Stands in for a mavutil connection: records sends, replays messages."""

    def __init__(self, messages=None, target_system=1, target_component=0,
                 on_send=None):
        self.target_system = target_system
        self.target_component = target_component
        self._queue = list(messages or [])
        self.sent = []
        self._on_send = on_send
        self.mav = types.SimpleNamespace(command_long_send=self._command_long_send)

    def _command_long_send(self, *args):
        self.sent.append(args)
        if self._on_send is not None:
            self._on_send(*args)

    def recv_match(self, blocking=False, timeout=None):
        return self._queue.pop(0) if self._queue else None


def vehicle_with(link):
    v = MavlinkVehicle("test:none")
    v.m = link
    return v


# --- arm / disarm on the hardware layer -------------------------------

@pytest.mark.parametrize("arm, force, params", [
    (True, False, (1, 0)),          # arm, the FC's checks decide
    (True, True, (1, 2989)),        # arm anyway
    (False, False, (0, 0)),
    (False, True, (0, 21196)),      # disarm anyway
])
def test_set_armed_sends_the_right_params(arm, force, params):
    link = FakeLink()
    v = vehicle_with(link)
    v.state["armed"] = arm                        # already there = instant ok
    assert v.set_armed(arm, force=force, timeout=0.2) == (True, "")
    assert link.sent[0][2] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    assert link.sent[0][4:6] == params


def test_arm_refusal_returns_the_fc_words():
    """The FC answers a refused arm with STATUSTEXT, not a useful ACK."""
    v = MavlinkVehicle("test:none")
    v.m = FakeLink(on_send=lambda *a: v._texts.append(
        (time.time(), "Arm: RC not found")))
    ok, reason = v.set_armed(True, timeout=0.3)
    assert ok is False
    assert reason == "Arm: RC not found"


def test_arm_refusal_reports_each_distinct_line_once():
    v = MavlinkVehicle("test:none")

    def refuse(*a):
        now = time.time()
        v._texts.append((now, "PreArm: Hardware safety switch"))
        v._texts.append((now, "PreArm: Hardware safety switch"))
        v._texts.append((now, "Arm: RC not found"))
        v._texts.append((now, "EKF3 IMU0 is using GPS"))   # unrelated chatter

    v.m = FakeLink(on_send=refuse)
    ok, reason = v.set_armed(True, timeout=0.3)
    assert ok is False
    assert reason == "PreArm: Hardware safety switch; Arm: RC not found"


def test_arm_refusal_falls_back_to_command_ack():
    v = MavlinkVehicle("test:none")
    v.m = FakeLink(on_send=lambda *a: v.state["acks"].__setitem__(
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED))
    ok, reason = v.set_armed(True, timeout=0.3)
    assert ok is False
    assert reason == "MAV_RESULT_TEMPORARILY_REJECTED"


def test_stale_ack_is_dropped_before_sending():
    """Only an ACK that arrives during the wait may explain a refusal."""
    v = vehicle_with(FakeLink())
    v.state["acks"][mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM] = \
        mavutil.mavlink.MAV_RESULT_ACCEPTED
    ok, reason = v.set_armed(True, timeout=0.2)
    assert ok is False
    assert "no armed-state change" in reason


def test_emergency_stop_is_still_a_force_disarm():
    link = FakeLink()
    v = vehicle_with(link)
    v.state["armed"] = False
    assert v.emergency_disarm(timeout=0.2) is True
    assert link.sent[-1][4:6] == (0, 21196)


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
