"""Unit tests for the driver: no aircraft, no broker, no network.

They cover the three things that broke on real hardware: arm/disarm and its
status reply, heartbeat provenance, and the NED->ENU attitude conversion.
"""

import math
import sys
import threading
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavutil  # noqa: E402

from cyberwave_edge_mavlink_driver import hardware  # noqa: E402
from cyberwave_edge_mavlink_driver.hardware import (  # noqa: E402
    MavlinkVehicle,
    enu_quaternion_from_ned_euler,
    is_vehicle_heartbeat,
)

ARM_HB = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED


class FakeHeartbeat:
    """Just enough of a pymavlink HEARTBEAT for the filter under test."""

    def __init__(self, sysid=1, compid=1, type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
                 autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                 base_mode=0, custom_mode=0):
        self._sysid, self._compid = sysid, compid
        self.type, self.autopilot = type, autopilot
        self.base_mode, self.custom_mode = base_mode, custom_mode

    def get_type(self):
        return "HEARTBEAT"

    def get_srcSystem(self):
        return self._sysid

    def get_srcComponent(self):
        return self._compid


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


# --- heartbeat provenance ---------------------------------------------

def test_vehicle_heartbeat_accepted():
    assert is_vehicle_heartbeat(FakeHeartbeat())


@pytest.mark.parametrize("kwargs", [
    # the real one seen on the bench link
    dict(compid=0, type=mavutil.mavlink.MAV_TYPE_ADSB,
         autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID, base_mode=4),
    dict(compid=0),                                       # wrong component
    dict(compid=mavutil.mavlink.MAV_COMP_ID_GIMBAL),      # a gimbal
    dict(type=mavutil.mavlink.MAV_TYPE_GCS),              # a GCS
    dict(type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER),
    dict(autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID),
])
def test_non_vehicle_heartbeats_rejected(kwargs):
    assert not is_vehicle_heartbeat(FakeHeartbeat(**kwargs))


def test_adsb_heartbeat_does_not_touch_state():
    """The bug: an ADSB heartbeat made armed read False while it was armed."""
    armed_hb = FakeHeartbeat(base_mode=ARM_HB | 1, custom_mode=0)
    adsb_hb = FakeHeartbeat(compid=0, type=mavutil.mavlink.MAV_TYPE_ADSB,
                            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            base_mode=4, custom_mode=99)
    v = vehicle_with(FakeLink([armed_hb, adsb_hb]))

    v.pump_once()
    assert v.state["armed"] is True
    assert v.state["mode"] == 0
    before = dict(v.state)

    assert v.pump_once() == "HEARTBEAT"          # consumed...
    assert v.state["armed"] is True              # ...but changed nothing
    assert v.state["mode"] == before["mode"]
    assert v.state["last_heartbeat"] == before["last_heartbeat"]


def test_foreign_heartbeat_logged_once(caplog):
    hbs = [FakeHeartbeat(compid=0, type=mavutil.mavlink.MAV_TYPE_ADSB,
                         autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID)
           for _ in range(3)]
    v = vehicle_with(FakeLink(hbs))
    with caplog.at_level("INFO", logger=hardware.__name__):
        for _ in range(3):
            v.pump_once()
    lines = [r for r in caplog.records if "non-vehicle HEARTBEAT" in r.message]
    assert len(lines) == 1
    assert "comp=0" in lines[0].getMessage()
    assert "MAV_TYPE_ADSB" in lines[0].getMessage()


def test_heartbeat_from_another_system_ignored():
    v = vehicle_with(FakeLink([FakeHeartbeat(sysid=42, base_mode=ARM_HB)],
                              target_system=1))
    v.pump_once()
    assert v.state["armed"] is False
    assert v.state["last_heartbeat"] == 0.0


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


# --- attitude conversion, NED/FRD -> ENU/FLU --------------------------

def rotate(q, v):
    """Rotate vector v by quaternion dict q (Hamilton, right-handed)."""
    w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    vx, vy, vz = v
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def approx(t):
    return pytest.approx(t, abs=1e-9)


def test_level_facing_north():
    """yaw 0 in NED = North = +Y in Cyberwave's ENU world."""
    q = enu_quaternion_from_ned_euler(0.0, 0.0, 0.0)
    assert q == {"w": approx(math.sqrt(0.5)), "x": approx(0.0),
                 "y": approx(0.0), "z": approx(math.sqrt(0.5))}
    assert rotate(q, (1, 0, 0)) == approx((0.0, 1.0, 0.0))   # nose to North


def test_roll_right_30_degrees():
    """roll +30 in ArduPilot = right wing DOWN. Facing East the body axes
    line up with the world, so this is a plain 30 degrees about +X."""
    q = enu_quaternion_from_ned_euler(math.radians(30), 0.0, math.pi / 2)
    assert q == {"w": approx(math.cos(math.radians(15))),
                 "x": approx(math.sin(math.radians(15))),
                 "y": approx(0.0), "z": approx(0.0)}
    left = rotate(q, (0, 1, 0))                     # body left wing...
    assert left[2] == approx(math.sin(math.radians(30)))     # ...goes UP


def test_pitch_nose_up_20_degrees():
    """pitch +20 in ArduPilot = nose UP; in ENU/FLU that is -20 about +Y."""
    q = enu_quaternion_from_ned_euler(0.0, math.radians(20), math.pi / 2)
    assert q == {"w": approx(math.cos(math.radians(10))),
                 "x": approx(0.0),
                 "y": approx(-math.sin(math.radians(10))),
                 "z": approx(0.0)}
    assert rotate(q, (1, 0, 0))[2] == approx(math.sin(math.radians(20)))


def test_yaw_east_is_world_x():
    """yaw 90 in NED = East = +X in ENU, and the quaternion is identity."""
    q = enu_quaternion_from_ned_euler(0.0, 0.0, math.radians(90))
    assert q == {"w": approx(1.0), "x": approx(0.0), "y": approx(0.0),
                 "z": approx(0.0)}
    assert rotate(q, (1, 0, 0)) == approx((1.0, 0.0, 0.0))


def test_quaternion_is_always_unit():
    for rpy in [(0.3, -0.2, 1.1), (-1.0, 0.5, -2.0), (0.0, 1.4, 3.0)]:
        q = enu_quaternion_from_ned_euler(*rpy)
        assert math.sqrt(sum(c * c for c in q.values())) == pytest.approx(1.0)


def test_attitude_quat_enu_uses_the_pure_function():
    v = vehicle_with(FakeLink())
    assert v.attitude_quat_enu() is None
    v.state["attitude"] = (0.1, 0.2, 0.3)
    assert v.attitude_quat_enu() == enu_quaternion_from_ned_euler(0.1, 0.2, 0.3)
