"""The verbs: arm/disarm and its refusal wording, then the parts of each
autopilot that differ. No aircraft, no broker, no network."""

import math
import sys
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavutil  # noqa: E402

from cyberwave_edge_mavlink_driver import px4  # noqa: E402
from cyberwave_edge_mavlink_driver.ardupilot import ArduPilot  # noqa: E402
from cyberwave_edge_mavlink_driver.link import MavlinkLink  # noqa: E402
from cyberwave_edge_mavlink_driver.px4 import PX4  # noqa: E402
from cyberwave_edge_mavlink_driver.vehicle import Vehicle, pick_vehicle  # noqa: E402

ARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
SET_MODE = mavutil.mavlink.MAV_CMD_DO_SET_MODE
COPTER_MODES = mavutil.mode_mapping_byname(mavutil.mavlink.MAV_TYPE_QUADROTOR)


class FakeMav:
    """A mavutil connection that records sends and lets a test answer them."""

    def __init__(self, on_send=None):
        self.target_system, self.target_component = 1, 1
        self.sent = []
        self.modes_asked = []
        self.params = {}
        self._on_send = on_send
        self.mav = types.SimpleNamespace(
            command_long_send=self._command_long_send,
            set_position_target_local_ned_send=lambda *a: self.sent.append(("vel",) + a),
            param_set_send=lambda sys, comp, name, value, kind: self.params.__setitem__(name, value),
            heartbeat_send=lambda *a: self.sent.append(("hb",) + a),
        )

    def _command_long_send(self, *args):
        self.sent.append(args)
        if self._on_send is not None:
            self._on_send(*args)

    def mode_mapping(self):
        return COPTER_MODES

    def set_mode(self, name):
        self.modes_asked.append(name)

    def commands(self):
        return [s[2] for s in self.sent if s[0] not in ("vel", "hb")]

    def heartbeats(self):
        return [s for s in self.sent if s[0] == "hb"]


def link_with(on_send=None):
    link = MavlinkLink("test:none")
    link.m = FakeMav(on_send)
    return link


# --- arm / disarm -----------------------------------------------------

@pytest.mark.parametrize("arm, force, params", [
    (True, False, (1, 0)),          # arm, the autopilot's checks decide
    (True, True, (1, 2989)),        # arm anyway
    (False, False, (0, 0)),
    (False, True, (0, 21196)),      # disarm anyway
])
def test_set_armed_sends_the_right_params(arm, force, params):
    link = link_with()
    link.state["armed"] = arm                        # already there = instant ok
    assert Vehicle(link).set_armed(arm, force=force, timeout=0.2) == (True, "")
    assert link.m.sent[0][2] == ARM
    assert link.m.sent[0][4:6] == params


def test_arm_refusal_returns_the_fc_words():
    """The autopilot answers a refused arm with STATUSTEXT, not a useful ACK."""
    link = link_with(lambda *a: link._texts.append((time.time(), "Arm: RC not found")))
    ok, reason = Vehicle(link).set_armed(True, timeout=0.3)
    assert ok is False
    assert reason == "Arm: RC not found"


def test_arm_refusal_reports_each_distinct_line_once():
    def refuse(*a):
        now = time.time()
        link._texts.append((now, "PreArm: Hardware safety switch"))
        link._texts.append((now, "PreArm: Hardware safety switch"))
        link._texts.append((now, "Arm: RC not found"))
        link._texts.append((now, "EKF3 IMU0 is using GPS"))   # unrelated chatter

    link = link_with(refuse)
    ok, reason = Vehicle(link).set_armed(True, timeout=0.3)
    assert ok is False
    assert reason == "PreArm: Hardware safety switch; Arm: RC not found"


def test_arm_refusal_falls_back_to_command_ack():
    link = link_with(lambda *a: link.state["acks"].__setitem__(
        ARM, mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED))
    ok, reason = Vehicle(link).set_armed(True, timeout=0.3)
    assert ok is False
    assert reason == "MAV_RESULT_TEMPORARILY_REJECTED"


def test_stale_ack_is_dropped_before_sending():
    """Only an ACK that arrives during the wait may explain a refusal."""
    link = link_with()
    link.state["acks"][ARM] = mavutil.mavlink.MAV_RESULT_ACCEPTED
    ok, reason = Vehicle(link).set_armed(True, timeout=0.2)
    assert ok is False
    assert "no armed-state change" in reason


def test_kill_is_a_force_disarm():
    link = link_with()
    assert Vehicle(link).kill() == (True, "")
    assert link.m.sent[-1][4:6] == (0, 21196)


def test_reboot_is_refused_while_armed():
    link = link_with()
    link.state["armed"] = True
    ok, reason = Vehicle(link).reboot()
    assert ok is False
    assert reason == "refused: the aircraft is armed"
    assert link.m.sent == []


def test_set_home_here_reports_a_refused_ack():
    link = link_with(lambda *a: link.state["acks"].__setitem__(
        mavutil.mavlink.MAV_CMD_DO_SET_HOME, mavutil.mavlink.MAV_RESULT_DENIED))
    ok, reason = Vehicle(link).set_home_here()
    assert ok is False
    assert reason == "MAV_RESULT_DENIED"


def test_in_air_prefers_the_landed_state_over_altitude():
    link = link_with()
    v = Vehicle(link)
    link.state["alt"] = 3.0
    assert v.in_air()
    link.state["landed"] = "on_ground"
    assert not v.in_air()


# --- ArduPilot --------------------------------------------------------

def test_ardupilot_mode_name_comes_from_the_heartbeat():
    link = link_with()
    link.state["mode"] = COPTER_MODES["GUIDED"]
    assert ArduPilot(link).mode_name() == "GUIDED"


def test_ardupilot_arms_out_of_land_via_guided():
    """LAND is not armable, so an arm request switches to GUIDED first."""
    link = link_with()
    link.state["mode"] = COPTER_MODES["LAND"]
    v = ArduPilot(link)

    def become(name):
        link.m.modes_asked.append(name)
        link.state["mode"] = COPTER_MODES[name]
    link.m.set_mode = become

    def arm(*a):
        link.state["armed"] = True
    link.m._on_send = arm

    assert v.set_armed(True, timeout=0.5) == (True, "")
    assert link.m.modes_asked == ["GUIDED"]


def test_ardupilot_mode_refusal_returns_the_fc_words():
    """A mode the vehicle will not take is refused on the text channel."""
    link = link_with()
    link.m.set_mode = lambda name: link._texts.append(
        (time.time(), "Flight mode change failed"))
    ok, reason = ArduPilot(link).set_mode("BRAKE", timeout=0.3)
    assert ok is False
    assert reason == "Flight mode change failed"


def test_ardupilot_mode_refusal_without_words_says_what_it_waited_for():
    link = link_with()
    ok, reason = ArduPilot(link).set_mode("BRAKE", timeout=0.3)
    assert ok is False
    assert reason.startswith("could not enter BRAKE within")


def test_ardupilot_hold_on_the_ground_does_nothing():
    link = link_with()
    assert ArduPilot(link).hold() == (True, "")
    assert link.m.modes_asked == []


def test_ardupilot_release_is_one_zero_setpoint():
    link = link_with()
    ArduPilot(link).release_sticks()
    assert link.m.sent == [("vel", 0, 1, 1, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                            0b0000011111000111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)]


# --- PX4 --------------------------------------------------------------

def test_px4_custom_mode_packing():
    assert px4.custom_mode(4, 2) == 0x02040000
    assert px4.custom_mode(4, 3) == 50593792
    assert px4.mode_name(0x06040000) == "AUTO.LAND"
    assert px4.mode_name(0x00060000) == "OFFBOARD"
    assert px4.mode_name(0x00030000) == "POSCTL"


def test_px4_takeoff_sets_the_mode_before_arming():
    def autopilot(sys, comp, cmd, conf, p1, p2, p3, *rest):
        if cmd == SET_MODE:
            link.state["mode"] = px4.custom_mode(int(p2), int(p3))
        elif cmd == ARM:
            link.state["armed"] = True
            link.state["landed"] = "in_air"

    link = link_with(autopilot)
    assert PX4(link).takeoff(3.0) == (True, "")
    assert link.m.commands() == [SET_MODE, ARM]
    assert link.m.params[b"MIS_TAKEOFF_ALT"] == 3.0
    assert all(math.isnan(p) for p in link.m.sent[0][7:])   # unused params are NaN


def test_px4_force_arm_is_downgraded_to_a_plain_arm():
    link = link_with(lambda *a: link.state.__setitem__("armed", True))
    assert PX4(link).set_armed(True, force=True) == (True, "")
    assert link.m.sent[0][4:6] == (1, 0)


def test_px4_ticks_a_gcs_heartbeat_once_a_second():
    """No GCS heartbeat, no STATUSTEXT: PX4 gates the text channel on it."""
    link = link_with()
    v = PX4(link)
    for _ in range(10):
        v.tick()
    assert len(link.m.heartbeats()) == 1
    assert link.m.heartbeats()[0][1] == mavutil.mavlink.MAV_TYPE_GCS
    v._heartbeat_at -= 1.0
    v.tick()
    assert len(link.m.heartbeats()) == 2


def test_ardupilot_never_heartbeats():
    """A GCS heartbeat here would engage ArduPilot's GCS failsafe."""
    link = link_with()
    v = ArduPilot(link)
    for _ in range(10):
        v.tick()
    assert link.m.sent == []


def test_px4_release_leaves_offboard_for_hold():
    def autopilot(sys, comp, cmd, conf, p1, p2, p3, *rest):
        if cmd == SET_MODE:
            link.state["mode"] = px4.custom_mode(int(p2), int(p3))

    link = link_with(autopilot)
    link.state["mode"] = px4.custom_mode(px4.MAIN["OFFBOARD"])
    PX4(link).release_sticks()
    assert link.m.sent[0][0] == "vel"
    assert link.state["mode"] == px4.custom_mode(4, 3)


# --- picking the backend ----------------------------------------------

@pytest.mark.parametrize("autopilot, cls", [
    (mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA, ArduPilot),
    (mavutil.mavlink.MAV_AUTOPILOT_PX4, PX4),
    (mavutil.mavlink.MAV_AUTOPILOT_GENERIC, ArduPilot),
])
def test_pick_vehicle_by_autopilot(autopilot, cls):
    link = link_with()
    link.autopilot = autopilot
    assert type(pick_vehicle(link)) is cls
