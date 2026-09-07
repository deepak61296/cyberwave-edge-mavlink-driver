"""The verbs: arm/disarm and its refusal wording, then the parts of each
autopilot that differ. No aircraft, no broker, no network."""

import math
import sys
import threading
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavutil  # noqa: E402

from cyberwave_edge_mavlink_driver import ardupilot, px4  # noqa: E402
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
        # what pymavlink noted from the heartbeat of the system we accepted
        self.sysid_state = {1: types.SimpleNamespace(mav_type=mavutil.mavlink.MAV_TYPE_QUADROTOR)}
        self.sent = []
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
    assert reason == "motors running"
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


def test_a_wait_gives_way_to_abort():
    v = Vehicle(link_with())
    v.abort.set()
    t0 = time.time()
    assert v._wait(lambda: False, 5.0) is False
    assert v._wait(lambda: True, 5.0) is True      # what already holds still counts
    ok, reason = v.set_armed(True, timeout=5.0)
    assert not ok
    assert time.time() - t0 < 0.5


# --- ArduPilot --------------------------------------------------------

def test_ardupilot_mode_name_comes_from_the_heartbeat():
    link = link_with()
    link.state["mode"] = COPTER_MODES["GUIDED"]
    assert ArduPilot(link).mode_name() == "GUIDED"


def test_ardupilot_mode_change_is_a_do_set_mode_to_the_accepted_system():
    link = link_with(lambda *a: link.state.__setitem__("mode", int(a[5])))
    assert ArduPilot(link).set_mode("GUIDED", timeout=0.5) == (True, "")
    assert link.m.sent == [(1, 1, SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                            COPTER_MODES["GUIDED"], 0.0, 0.0, 0.0, 0.0, 0.0)]


def test_ardupilot_mode_table_follows_the_accepted_system():
    """pymavlink's own table follows whichever heartbeat it saw first."""
    link = link_with()
    link.m.sysid_state[1].mav_type = mavutil.mavlink.MAV_TYPE_GROUND_ROVER
    v = ArduPilot(link)
    assert "BRAKE" not in v.modes
    link.state["mode"] = mavutil.mode_mapping_byname(mavutil.mavlink.MAV_TYPE_GROUND_ROVER)["GUIDED"]
    assert v.mode_name() == "GUIDED"


def test_ardupilot_arms_out_of_land_via_guided():
    """LAND is not armable, so an arm request switches to GUIDED first."""
    link = link_with()
    link.state["mode"] = COPTER_MODES["LAND"]

    def autopilot(sys, comp, cmd, conf, p1, p2, p3, *rest):
        if cmd == SET_MODE:
            link.state["mode"] = int(p2)
        elif cmd == ARM:
            link.state["armed"] = True
    link.m._on_send = autopilot

    assert ArduPilot(link).set_armed(True, timeout=0.5) == (True, "")
    assert link.m.commands() == [SET_MODE, ARM]
    assert link.m.sent[0][5] == COPTER_MODES["GUIDED"]


def test_ardupilot_mode_refusal_returns_the_fc_words():
    """A mode the vehicle will not take is refused on the text channel."""
    link = link_with(lambda *a: link._texts.append(
        (time.time(), "Flight mode change failed")))
    ok, reason = ArduPilot(link).set_mode("BRAKE", timeout=0.3)
    assert ok is False
    assert reason == "Flight mode change failed"


def test_ardupilot_mode_refusal_without_words_says_what_it_waited_for():
    link = link_with()
    ok, reason = ArduPilot(link).set_mode("BRAKE", timeout=0.3)
    assert ok is False
    assert reason.startswith("could not enter BRAKE within")


def test_ardupilot_takeoff_is_guided_then_arm_then_nav_takeoff(monkeypatch):
    monkeypatch.setattr(ardupilot, "ARM_TRY_S", 0.2)
    monkeypatch.setattr(ardupilot, "ARM_RETRY_S", 2.0)
    asked = []

    def autopilot(sys, comp, cmd, conf, p1, p2, p3, *rest):
        if cmd == SET_MODE:
            link.state["mode"] = int(p2)
        elif cmd == ARM:
            asked.append(cmd)
            if len(asked) == 1:     # pre-arm refuses the first ask with words, no bit
                link._texts.append((time.time(), "PreArm: waiting for EKF"))
            else:
                link.state["armed"] = True
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            link.state["acks"][cmd] = mavutil.mavlink.MAV_RESULT_ACCEPTED
            link.state["landed"] = "in_air"

    link = link_with(autopilot)
    assert ArduPilot(link).takeoff(3.0) == (True, "")
    assert link.m.commands() == [SET_MODE, ARM, ARM, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF]
    assert link.m.sent[0][5] == COPTER_MODES["GUIDED"]
    assert link.m.sent[-1][10] == 3.0                # NAV_TAKEOFF param7 is the altitude


def test_ardupilot_takeoff_waits_for_the_aircraft_to_leave_the_ground(monkeypatch):
    """The ack says the command was taken, not that anything moved."""
    monkeypatch.setattr(ardupilot, "AIRBORNE_S", 0.5)

    def autopilot(sys, comp, cmd, conf, p1, p2, p3, *rest):
        if cmd == SET_MODE:
            link.state["mode"] = int(p2)
        elif cmd == ARM:
            link.state["armed"] = True
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            link.state["acks"][cmd] = mavutil.mavlink.MAV_RESULT_ACCEPTED

    link = link_with(autopilot)
    assert ArduPilot(link).takeoff(3.0) == (False, "armed but never left the ground")
    link.state["alt"] = 1.2                      # no landed state, altitude alone
    assert ArduPilot(link).takeoff(3.0) == (True, "")


def test_ardupilot_takeoff_gives_way_to_a_kill_while_it_climbs():
    def autopilot(sys, comp, cmd, conf, p1, p2, p3, *rest):
        if cmd == SET_MODE:
            link.state["mode"] = int(p2)
        elif cmd == ARM:
            link.state["armed"] = True
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            link.state["acks"][cmd] = mavutil.mavlink.MAV_RESULT_ACCEPTED

    link = link_with(autopilot)
    v = ArduPilot(link)
    threading.Timer(0.2, v.abort.set).start()
    t0 = time.time()
    ok, _ = v.takeoff(3.0)
    assert not ok
    assert time.time() - t0 < 2.0                # not the full AIRBORNE_S wait


def test_ardupilot_reports_the_altitude_it_asked_for():
    link = link_with()
    link.state["alt"] = 0.7
    assert ArduPilot(link).takeoff_altitude(3.0) == 3.0


def test_ardupilot_hold_on_the_ground_does_nothing():
    link = link_with()
    assert ArduPilot(link).hold() == (True, "")
    assert link.m.sent == []


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
            link.state["alt"] = 2.8

    link = link_with(autopilot)
    assert PX4(link).takeoff(3.0) == (True, "")
    assert link.m.commands() == [SET_MODE, ARM]
    assert link.m.params[b"MIS_TAKEOFF_ALT"] == 3.0
    assert all(math.isnan(p) for p in link.m.sent[0][7:])   # unused params are NaN


def test_px4_takeoff_waits_for_the_altitude_not_just_the_landed_state(monkeypatch):
    def autopilot(sys, comp, cmd, conf, p1, p2, p3, *rest):
        if cmd == SET_MODE:
            link.state["mode"] = px4.custom_mode(int(p2), int(p3))
        elif cmd == ARM:
            link.state["armed"] = True
            link.state["landed"] = "in_air"   # PX4 says this from 0 m up

    link = link_with(autopilot)
    monkeypatch.setattr(px4, "TAKEOFF_CONFIRM_S", 0.5)
    ok, reason = PX4(link).takeoff(3.0)
    assert not ok
    assert "still climbing" in reason


def test_px4_reports_the_altitude_it_reached():
    link = link_with()
    v = PX4(link)
    assert v.takeoff_altitude(3.0) == 3.0        # still on the ground, nothing to report
    link.state["landed"], link.state["alt"] = "in_air", 2.83
    assert v.takeoff_altitude(3.0) == 2.83


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


def test_px4_prepare_streams_zeros_then_asks_for_offboard():
    """OFFBOARD only engages with a setpoint stream already running."""
    def autopilot(sys, comp, cmd, conf, p1, p2, p3, *rest):
        if cmd == SET_MODE:
            link.state["mode"] = px4.custom_mode(int(p2), int(p3))

    link = link_with(autopilot)
    link.state["armed"] = True
    PX4(link).prepare_sticks()
    assert [s[0] for s in link.m.sent[:5]] == ["vel"] * 5
    assert link.m.commands() == [SET_MODE]
    assert link.state["mode"] == px4.custom_mode(px4.MAIN["OFFBOARD"])


def test_px4_prepare_does_nothing_disarmed_or_already_offboard():
    link = link_with()
    PX4(link).prepare_sticks()
    assert link.m.sent == []
    link.state["armed"] = True
    link.state["mode"] = px4.custom_mode(px4.MAIN["OFFBOARD"])
    PX4(link).prepare_sticks()
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


def test_px4_release_keeps_the_zeros_flowing_until_hold_shows():
    """The offboard-loss failsafe fires a second after the stream stops."""
    link = link_with()
    link.state["mode"] = px4.custom_mode(px4.MAIN["OFFBOARD"])

    def hold_later():
        time.sleep(0.3)
        link.state["mode"] = px4.custom_mode(4, 3)
    threading.Thread(target=hold_later).start()

    PX4(link).release_sticks()
    assert link.m.commands() == [SET_MODE]
    zeros = [s for s in link.m.sent if s[0] == "vel"]
    assert len(zeros) >= 4                      # kept up while Hold was pending
    assert link.m.sent[-1][0] == "vel"          # and outlived the mode ask
    assert link.state["mode"] == px4.custom_mode(4, 3)


def test_px4_setpoints_use_body_ned():
    # frame 9 comes back as "coordinate frame 9 unsupported" and is dropped
    link = link_with()
    PX4(link).send_velocity_body(1.0, 0, 0, 0)
    assert link.m.sent[0][4] == mavutil.mavlink.MAV_FRAME_BODY_NED


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


# --- PX4 gimbal, home point and compass -------------------------------

CONFIGURE = mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
PITCHYAW = mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW
SET_HOME = mavutil.mavlink.MAV_CMD_DO_SET_HOME
CALIBRATE = mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION
ACCEPTED = mavutil.mavlink.MAV_RESULT_ACCEPTED


class GimbalMav(FakeMav):
    """FakeMav plus what these verbs use: a COMMAND_INT, the gimbal manager
    message, our own ids, and the message cache the pump keeps."""

    def __init__(self, on_send=None):
        super().__init__(on_send)
        self.messages = {}          # last of each type, as pymavlink keeps them
        self.ints = []
        self.attitudes = []
        self.mav.srcSystem, self.mav.srcComponent = 255, 190
        self.mav.command_int_send = self._command_int_send
        self.mav.gimbal_manager_set_attitude_send = lambda *a: self.attitudes.append(a)

    def _command_int_send(self, sys, comp, frame, cmd, current, autocontinue, *rest):
        self.ints.append((sys, comp, frame, cmd, current, autocontinue) + rest)
        if self._on_send is not None:
            self._on_send(sys, comp, cmd, 0, *rest)


def gimbal_link(results=None):
    """A link whose autopilot answers each command with the result we choose.

    None means it says nothing at all, which is what a PX4 with no gimbal
    module does: the commander leaves both gimbal commands to it.
    """
    results = {} if results is None else results

    def autopilot(sys, comp, cmd, conf, *params):
        result = results.get(cmd, ACCEPTED)
        if result is not None:
            link.state["acks"][cmd] = result

    link = MavlinkLink("test:none")
    link.m = GimbalMav(autopilot)
    return link


def test_px4_gimbal_point_takes_control_before_it_points():
    link = gimbal_link()
    PX4(link).gimbal_point(-30.0, 45.0, False)
    assert link.m.commands() == [CONFIGURE, PITCHYAW]
    assert link.m.sent[0][4:6] == (255, 190)    # our ids, or the next one is denied
    angle = link.m.sent[1]
    assert angle[4:6] == (-30.0, 45.0)
    assert math.isnan(angle[6]) and math.isnan(angle[7])    # no rate beside an angle
    assert angle[8] == 0                                    # body frame


def test_px4_gimbal_point_locks_the_frame_when_absolute():
    link = gimbal_link()
    PX4(link).gimbal_point(-15.0, 90.0, True)
    assert link.m.sent[1][8] == px4.EARTH_FRAME == 24


def test_px4_gimbal_point_ignores_a_slew_duration():
    """DJI takes a rotation time; the gimbal manager has no field for one."""
    link = gimbal_link()
    PX4(link).gimbal_point(-20.0, 0.0, False, duration_s=2.0)
    assert link.m.sent[1][4:6] == (-20.0, 0.0)
    assert link.m.commands() == [CONFIGURE, PITCHYAW]


def test_px4_gimbal_point_says_not_supported_when_nothing_answers(monkeypatch):
    monkeypatch.setattr(px4, "GIMBAL_ACK_S", 0.2)
    link = gimbal_link({CONFIGURE: None})
    with pytest.raises(px4.VehicleError, match="not supported on this vehicle"):
        PX4(link).gimbal_point(-30.0, 0.0, False)


def test_px4_gimbal_point_passes_the_fc_verdict_on():
    link = gimbal_link({PITCHYAW: mavutil.mavlink.MAV_RESULT_DENIED})
    with pytest.raises(px4.VehicleError, match="MAV_RESULT_DENIED"):
        PX4(link).gimbal_point(0.0, 0.0, False)


def test_px4_gimbal_rate_sends_a_rate_and_no_angle():
    """The command's rate fields are useless here: PX4 reads a NaN angle as
    zero and every refresh drags the mount back to centre."""
    link = gimbal_link()
    v = PX4(link)
    v.gimbal_rate(-10.0, 20.0)
    v.gimbal_rate(0.0, 0.0)
    assert link.m.commands() == [CONFIGURE]         # control is claimed once
    q, _, pitch_rate, yaw_rate = link.m.attitudes[0][4:8]
    assert all(math.isnan(x) for x in q)
    assert pitch_rate == pytest.approx(math.radians(-10.0))
    assert yaw_rate == pytest.approx(math.radians(20.0))
    assert link.m.attitudes[1][6:8] == (0.0, 0.0)


def test_px4_gimbal_rate_asks_for_control_once_when_there_is_no_gimbal(monkeypatch):
    """A stick refreshes at 10 Hz; a two second timeout on each would stall."""
    monkeypatch.setattr(px4, "GIMBAL_ACK_S", 0.2)
    link = gimbal_link({CONFIGURE: None})
    v = PX4(link)
    for _ in range(3):
        with pytest.raises(px4.VehicleError, match="not supported on this vehicle"):
            v.gimbal_rate(-10.0, 0.0)
    assert link.m.commands() == [CONFIGURE]
    assert link.m.attitudes == []


@pytest.mark.parametrize("q, degrees", [
    ((0.89239907, 0.09904575, -0.23911758, 0.36964384), (-30.0, 45.0)),
    ((0.70105737, 0.09229596, -0.09229596, 0.70105737), (-15.0, 90.0)),
    ((1.0, 0.0, 0.0, 0.0), (0.0, 0.0)),
])
def test_px4_gimbal_attitude_reads_the_quaternion_back(q, degrees):
    """The quaternions are the ones PX4 sent for those angles on the bench."""
    link = gimbal_link()
    link.m.messages["GIMBAL_DEVICE_ATTITUDE_STATUS"] = types.SimpleNamespace(q=q)
    assert PX4(link).gimbal_attitude() == degrees


def test_pitch_yaw_degrees_survives_straight_down():
    """asin of a hair past -1 raises, and a float quaternion gets there."""
    pitch, _ = px4.pitch_yaw_degrees((0.7071069, 0.0, -0.7071069, 0.0))
    assert pitch == -90.0


def test_px4_gimbal_attitude_falls_back_to_mount_orientation():
    link = gimbal_link()
    link.m.messages["MOUNT_ORIENTATION"] = types.SimpleNamespace(pitch=-12.3456, yaw=7.0)
    assert PX4(link).gimbal_attitude() == (-12.35, 7.0)


def test_px4_gimbal_attitude_is_none_with_no_mount_talking():
    assert PX4(gimbal_link()).gimbal_attitude() is None


def test_px4_set_home_goes_as_a_command_int():
    """A COMMAND_LONG carries the latitude in a float32, which moves home."""
    link = gimbal_link()
    PX4(link).set_home(47.3977435, 8.5455937, 489.4)
    sent, = link.m.ints
    assert sent[2:4] == (mavutil.mavlink.MAV_FRAME_GLOBAL, SET_HOME)
    assert sent[6] == 0                 # param1: 0 takes the point in the message
    assert sent[10:12] == (473977435, 85455937)
    assert sent[12] == pytest.approx(489.4)


def test_px4_set_home_uses_the_aircraft_height_when_none_is_given():
    """PX4 denies a home point with no altitude, so we send the one we have."""
    link = gimbal_link()
    link.m.messages["GLOBAL_POSITION_INT"] = types.SimpleNamespace(alt=489409)
    PX4(link).set_home(47.0, 8.0, None)
    assert link.m.ints[0][12] == pytest.approx(489.409)


def test_px4_set_home_with_no_altitude_and_no_fix_refuses():
    with pytest.raises(px4.VehicleError, match="no position fix"):
        PX4(gimbal_link()).set_home(47.0, 8.0, None)


def test_px4_set_home_passes_the_refusal_on():
    link = gimbal_link({SET_HOME: mavutil.mavlink.MAV_RESULT_DENIED})
    with pytest.raises(px4.VehicleError, match="MAV_RESULT_DENIED"):
        PX4(link).set_home(47.0, 8.0, 489.0)


def test_px4_compass_calibration_starts_the_magnetometer():
    link = gimbal_link()
    PX4(link).compass_calibration(True)
    assert link.m.commands() == [CALIBRATE]
    assert link.m.sent[0][4:6] == (0, 1)        # param2 = 1 is the mag


def test_px4_compass_calibration_is_refused_with_the_motors_running():
    link = gimbal_link()
    link.state["armed"] = True
    with pytest.raises(px4.VehicleError, match="motors running"):
        PX4(link).compass_calibration(True)
    assert link.m.sent == []


def test_px4_compass_cancel_goes_again_until_px4_takes_it():
    """A cancel that lands in the middle of a sampling step is answered
    TEMPORARILY_REJECTED by the commander and stops nothing."""
    tries = []

    def autopilot(sys, comp, cmd, conf, *params):
        tries.append(params[:2])
        link.state["acks"][cmd] = mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED
        if len(tries) == 3:
            link._texts.append((time.time(), "[cal] calibration cancelled"))

    link = MavlinkLink("test:none")
    link.m = GimbalMav(autopilot)
    PX4(link).compass_calibration(False)
    assert tries == [(0, 0)] * 3


def test_px4_compass_cancel_gives_up_with_the_fc_verdict(monkeypatch):
    monkeypatch.setattr(px4, "CALIBRATION_TRIES", 2)
    link = gimbal_link({CALIBRATE: mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED})
    with pytest.raises(px4.VehicleError, match="MAV_RESULT_TEMPORARILY_REJECTED"):
        PX4(link).compass_calibration(False)
