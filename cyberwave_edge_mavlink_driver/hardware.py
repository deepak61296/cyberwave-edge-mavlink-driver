"""MAVLink hardware layer — a headless GCS wrapping pymavlink.

One rule governs this file: exactly ONE thread (the driver's pump) calls
recv; every other thread only sends. Command methods therefore wait on
shared state that the pump keeps fresh, never on the socket itself.

Lessons baked in from the SITL spike (see ARCHITECTURE-NOTES.md session 3):
  - mode changes are verified via HEARTBEAT.custom_mode and retried
  - commands are trusted only on COMMAND_ACK
  - raw serial0 streams nothing until REQUEST_DATA_STREAM
"""

import logging
import math
import time
from typing import Any, Optional

from pymavlink import mavutil

logger = logging.getLogger(__name__)

VEL_MASK = 0b0000011111000111  # velocity + yaw-rate control


class MavlinkVehicle:
    """Connection + state cache + command surface for one autopilot."""

    def __init__(self, connection: str) -> None:
        self.connection_string = connection
        self.m: Optional[Any] = None
        # written ONLY by pump_once(), read by everyone
        self.state: dict[str, Any] = {
            "armed": False, "mode": None, "alt": 0.0,
            "ned": None, "attitude": None, "acks": {}, "last_heartbeat": 0.0,
            "servo_pwm": None,
        }

    # -- connection ----------------------------------------------------

    def connect(self, timeout: float = 60.0) -> None:
        logger.info("MAVLink connecting to %s", self.connection_string)
        self.m = mavutil.mavlink_connection(self.connection_string)
        # Behind mavlink-router the first heartbeat can carry sysid 0
        # (seen on the Pi, 2026-08-30); accepting it would turn every
        # command into a broadcast. Wait for a real system id.
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.m.wait_heartbeat(timeout=5) is not None \
                    and self.m.target_system != 0:
                break
        if self.m.target_system == 0:
            raise ConnectionError(f"no usable heartbeat on {self.connection_string}")
        logger.info("heartbeat: sys=%s comp=%s", self.m.target_system, self.m.target_component)
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # -- pump (call from ONE thread only) ------------------------------

    def pump_once(self, timeout: float = 1.0) -> Optional[str]:
        """Receive one message, fold it into state, return its type."""
        msg = self.m.recv_match(blocking=True, timeout=timeout)
        if msg is None:
            return None
        k = msg.get_type()
        s = self.state
        if k == "HEARTBEAT":
            s["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            s["mode"] = msg.custom_mode
            s["last_heartbeat"] = time.time()
        elif k == "COMMAND_ACK":
            s["acks"][msg.command] = msg.result
        elif k == "GLOBAL_POSITION_INT":
            s["alt"] = msg.relative_alt / 1000.0
        elif k == "LOCAL_POSITION_NED":
            s["ned"] = (msg.x, msg.y, msg.z)
        elif k == "ATTITUDE":
            s["attitude"] = (msg.roll, msg.pitch, msg.yaw)
        elif k == "SERVO_OUTPUT_RAW":
            s["servo_pwm"] = (msg.servo1_raw, msg.servo2_raw,
                              msg.servo3_raw, msg.servo4_raw)
        elif k == "STATUSTEXT":
            logger.info("[fc] %s", msg.text)
        return k

    # -- conversions ---------------------------------------------------

    def position_enu(self) -> Optional[tuple]:
        """NED -> Cyberwave Z-up (x=east, y=north, z=up, clamped >= 0)."""
        ned = self.state["ned"]
        if ned is None:
            return None
        return (ned[1], ned[0], max(0.0, -ned[2]))

    def attitude_quat_enu(self) -> Optional[dict]:
        """NED euler -> ENU quaternion (w,x,y,z). Visual check pending."""
        att = self.state["attitude"]
        if att is None:
            return None
        roll, pitch, yaw = att[0], -att[1], math.pi / 2 - att[2]
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return {
            "w": cr * cp * cy + sr * sp * sy,
            "x": sr * cp * cy - cr * sp * sy,
            "y": cr * sp * cy + sr * cp * sy,
            "z": cr * cp * sy - sr * sp * cy,
        }

    # -- commands (safe from any thread; never recv) -------------------

    def _wait_ack(self, cmd: int, timeout: float = 5.0) -> Optional[int]:
        end = time.time() + timeout
        while time.time() < end:
            if cmd in self.state["acks"]:
                return self.state["acks"].pop(cmd)
            time.sleep(0.05)
        return None

    def ensure_mode(self, name: str, timeout: float = 30.0) -> bool:
        want = self.m.mode_mapping()[name]
        end = time.time() + timeout
        while time.time() < end:
            self.m.set_mode(name)
            time.sleep(1.0)
            if self.state["mode"] == want:
                logger.info("mode %s confirmed", name)
                return True
        logger.error("could not enter mode %s", name)
        return False

    def arm(self, timeout: float = 120.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if self.state["armed"]:
                logger.info("armed")
                return True
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
            time.sleep(3.0)
        logger.error("arming timed out (PreArm reasons in [fc] log lines)")
        return False

    def takeoff(self, altitude: float) -> bool:
        if not self.ensure_mode("GUIDED"):
            return False
        if not self.arm():
            return False
        for _ in range(5):
            self.m.mav.command_long_send(
                self.m.target_system, self.m.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
            if self._wait_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF) == \
                    mavutil.mavlink.MAV_RESULT_ACCEPTED:
                logger.info("takeoff accepted -> %.1f m", altitude)
                return True
            time.sleep(2.0)
        logger.error("NAV_TAKEOFF never accepted")
        return False

    def land(self) -> bool:
        return self.ensure_mode("LAND")

    def return_to_home(self) -> bool:
        return self.ensure_mode("RTL")

    def emergency_disarm(self) -> None:
        """Force-disarm regardless of state (magic 21196 = force)."""
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)
        logger.warning("EMERGENCY force-disarm sent")

    def send_velocity_body(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        """One body-frame velocity setpoint. Callers stream this at ~10 Hz."""
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            VEL_MASK, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate)

    def disconnect(self) -> None:
        if self.m is not None:
            self.m.close()
