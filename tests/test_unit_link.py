"""The MAVLink link: heartbeat provenance, the state cache, the NED->ENU
attitude conversion. No aircraft, no broker, no network."""

import math
import sys
import time
import types
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pymavlink import mavutil  # noqa: E402

from cyberwave_edge_mavlink_driver import link as link_module  # noqa: E402
from cyberwave_edge_mavlink_driver.link import (  # noqa: E402
    MavlinkLink,
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


class FakeMessage:
    """Any other message: a type name plus fields."""

    def __init__(self, type, **fields):
        self._type = type
        self.__dict__.update(fields)

    def get_type(self):
        return self._type


class FakeMav:
    """Stands in for a mavutil connection: records sends, replays messages."""

    def __init__(self, messages=None, target_system=1, target_component=1,
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


def link_with(mav):
    link = MavlinkLink("test:none")
    link.m = mav
    return link


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
    link = link_with(FakeMav([armed_hb, adsb_hb]))

    link.pump_once()
    assert link.state["armed"] is True
    assert link.state["mode"] == 0
    before = dict(link.state)

    assert link.pump_once() == "HEARTBEAT"          # consumed...
    assert link.state["armed"] is True              # ...but changed nothing
    assert link.state["mode"] == before["mode"]
    assert link.state["last_heartbeat"] == before["last_heartbeat"]


def test_foreign_heartbeat_logged_once(caplog):
    hbs = [FakeHeartbeat(compid=0, type=mavutil.mavlink.MAV_TYPE_ADSB,
                         autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID)
           for _ in range(3)]
    link = link_with(FakeMav(hbs))
    with caplog.at_level("INFO", logger=link_module.__name__):
        for _ in range(3):
            link.pump_once()
    lines = [r for r in caplog.records if "non-vehicle HEARTBEAT" in r.message]
    assert len(lines) == 1
    assert "comp=0" in lines[0].getMessage()
    assert "MAV_TYPE_ADSB" in lines[0].getMessage()


def test_heartbeat_from_another_system_ignored():
    link = link_with(FakeMav([FakeHeartbeat(sysid=42, base_mode=ARM_HB)],
                             target_system=1))
    link.pump_once()
    assert link.state["armed"] is False
    assert link.state["last_heartbeat"] == 0.0


# --- the state cache --------------------------------------------------

def test_connected_means_a_recent_vehicle_heartbeat():
    link = link_with(FakeMav([FakeHeartbeat()]))
    assert not link.connected()
    link.pump_once()
    assert link.connected()
    link.state["last_heartbeat"] = time.time() - 10
    assert not link.connected()


@pytest.mark.parametrize("landed_state, expected", [
    (mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, "on_ground"),
    (mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, "in_air"),
    (mavutil.mavlink.MAV_LANDED_STATE_TAKEOFF, "in_air"),
    (mavutil.mavlink.MAV_LANDED_STATE_LANDING, "in_air"),
    (mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED, None),
])
def test_extended_sys_state_folds_into_landed(landed_state, expected):
    link = link_with(FakeMav([FakeMessage("EXTENDED_SYS_STATE",
                                          landed_state=landed_state)]))
    assert link.pump_once() == "EXTENDED_SYS_STATE"
    assert link.state["landed"] == expected


def test_attitude_is_timestamped():
    link = link_with(FakeMav([FakeMessage("ATTITUDE", roll=0.1, pitch=0.2, yaw=0.3)]))
    link.pump_once()
    assert link.state["attitude"] == (0.1, 0.2, 0.3)
    assert time.time() - link.state["last_attitude"] < 1.0


def test_send_command_pads_to_seven_and_drops_the_stale_ack():
    mav = FakeMav()
    link = link_with(mav)
    link.state["acks"][400] = mavutil.mavlink.MAV_RESULT_ACCEPTED
    link.send_command(400, 1, 0)
    assert mav.sent[0] == (1, 1, 400, 0, 1, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert 400 not in link.state["acks"]
    assert link.wait_ack(400, timeout=0.1) is None


def test_send_command_can_fill_with_nan():
    mav = FakeMav()
    link_with(mav).send_command(176, 1, 4, 2, fill=float("nan"))
    assert all(math.isnan(p) for p in mav.sent[0][7:])


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
    link = link_with(FakeMav())
    assert link.attitude_quat_enu() is None
    link.state["attitude"] = (0.1, 0.2, 0.3)
    assert link.attitude_quat_enu() == enu_quaternion_from_ned_euler(0.1, 0.2, 0.3)


def test_position_enu_swaps_axes_and_never_goes_underground():
    link = link_with(FakeMav())
    assert link.position_enu() is None
    link.state["ned"] = (2.0, 3.0, -1.5)
    assert link.position_enu() == (3.0, 2.0, 1.5)
    link.state["ned"] = (0.0, 0.0, 0.2)
    assert link.position_enu() == (0.0, 0.0, 0.0)
