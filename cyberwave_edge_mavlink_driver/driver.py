"""CyberwaveEdgeMavlinkDriver — the bridge between the Cyberwave drone
command contract and any MAVLink autopilot (ArduPilot / PX4, SITL or real).

Four loops, mirroring the architecture that flew as bridge_v0:

  pump      (thread, sole MAVLink reader): fold messages into state,
            publish position + attitude to the twin's MQTT topics at 10 Hz
  commands  (thread): consume the command queue; discrete commands
            (takeoff/land/RTH/...) block here, never in the MQTT callback
  streamer  (thread): while a continuous command is fresh (< 0.5 s,
            per the DJI contract), emit body-frame velocity setpoints
            at 10 Hz; on expiry, zero the sticks once (dead-man)
  MQTT      (paho's thread): parse envelope, filter source_type, enqueue

Command vocabulary implemented (subset of dji/mini-4-pro contract v1,
plus arm/disarm, which that contract has no verb for):
  discrete:   takeoff, land, return_to_home, stop, emergency_stop,
              arm, disarm
  continuous: move_forward, move_backward, strafe_left, strafe_right,
              turn_left, turn_right, ascend, descend
"""

import json
import logging
import math
import os
import queue
import threading
import time
from pathlib import Path
from typing import Any, Optional

from cyberwave import Cyberwave

from cyberwave_edge_mavlink_driver.hardware import MavlinkVehicle

logger = logging.getLogger(__name__)

CONTINUOUS_TIMEOUT_S = 0.5   # DJI contract: zero sticks if no refresh in 500 ms
TELEMETRY_HZ = 10.0
ARM_CONFIRM_S = 5.0          # how long to wait for the armed bit to agree
STATE_PERIOD_S = 1.0         # armed/mode heartbeat to the twin (also on change)
DEFAULT_SPEED = 1.0          # m/s for continuous locomotion
DEFAULT_YAW_RATE = 0.5       # rad/s for turns
DEFAULT_TAKEOFF_ALT = 2.0

# Prop-joint animation (twin assets with prop_N_joint continuous joints).
# The viewer renders joint POSITIONS only, so we integrate a PWM-scaled
# visual spin rate into wrapped angles — legible spin, not true prop RPM.
PROP_JOINTS = ("prop_1_joint", "prop_2_joint", "prop_3_joint", "prop_4_joint")
# Directions match the asset's URDF (prop_1/2 carry the CCW mesh, prop_3/4
# the CW mesh — donor rotor roles), which is also ArduPilot's quad-X motor
# order: M1 front-right CCW, M2 rear-left CCW, M3 front-left CW, M4
# rear-right CW. CCW viewed from above = positive rotation about +Z.
PROP_DIRS = (1, 1, -1, -1)
PROP_VISUAL_MAX_RAD_S = 20.0

# command -> body-frame (vx, vy, vz, yaw_rate) unit vector
CONTINUOUS: dict[str, tuple] = {
    "move_forward":  (1, 0, 0, 0),
    "move_backward": (-1, 0, 0, 0),
    "strafe_right":  (0, 1, 0, 0),
    "strafe_left":   (0, -1, 0, 0),
    "descend":       (0, 0, 1, 0),   # NED: +z is down
    "ascend":        (0, 0, -1, 0),
    "turn_right":    (0, 0, 0, 1),
    "turn_left":     (0, 0, 0, -1),
}


class CyberwaveEdgeMavlinkDriver:
    """MAVLink edge driver speaking the standard Cyberwave drone contract."""

    def __init__(
        self,
        twin_uuid: str,
        api_key: str,
        twin_json_file: Optional[str] = None,
        child_uuids: Optional[list[str]] = None,
        connection: Optional[str] = None,
    ) -> None:
        self.twin_uuid = twin_uuid
        self.api_key = api_key
        self.twin_json_file = Path(twin_json_file) if twin_json_file else None
        self.child_uuids = child_uuids or []
        # Contract: "Only source_type tele is executed on the aircraft."
        # sim_tele is opt-in for SITL rigs (CYBERWAVE_ACCEPT_SIM_TELE=1);
        # a driver pointed at real hardware must never fly simulator traffic.
        self.accept_sim_tele = os.environ.get("CYBERWAVE_ACCEPT_SIM_TELE", "0") == "1"

        # Scaffold pattern: when Edge Core launches the driver, per-device
        # runtime config arrives via metadata.edge_configs in the twin JSON.
        # Env var wins for standalone/dev runs.
        self.edge_configs: dict[str, Any] = self._load_edge_configs()
        conn = (
            connection
            or os.environ.get("MAVLINK_CONNECTION")
            or self.edge_configs.get("mavlink_connection")
            or "tcp:127.0.0.1:5760"
        )
        self.vehicle = MavlinkVehicle(conn)

        self._prop_angles = [0.0] * 4
        self._prop_last = time.time()
        self._state_last: tuple = (None, None)   # (armed, mode_name) last sent
        self._state_at = 0.0

        self._cmd_queue: "queue.Queue[dict]" = queue.Queue()
        self._cont_lock = threading.Lock()
        self._cont_vec: Optional[tuple] = None   # (vx, vy, vz, yaw_rate)
        self._cont_at = 0.0
        self._stop = threading.Event()

        self._cw = Cyberwave()
        self._mq = self._cw.mqtt

    def _load_edge_configs(self) -> dict[str, Any]:
        if not self.twin_json_file:
            return {}
        try:
            meta = json.loads(self.twin_json_file.read_text()).get("metadata") or {}
            configs = meta.get("edge_configs") or {}
            if configs:
                logger.info("edge_configs: %s", configs)
            return configs
        except Exception:
            logger.exception("could not read twin JSON at %s", self.twin_json_file)
            return {}

    # ------------------------------------------------------------------
    # MQTT side
    # ------------------------------------------------------------------

    def _subscribe_commands(self) -> None:
        # Public SDK helper — subscribes to cyberwave/twin/{uuid}/command
        # (with the client's topic prefix applied, matching the backend).
        self._mq.subscribe_command_message(self.twin_uuid, self._on_command)
        logger.info("subscribed to command topic for twin %s", self.twin_uuid)

    def _on_command(self, msg: Any) -> None:
        """paho thread: validate + enqueue, never block."""
        try:
            env = msg if isinstance(msg, dict) else json.loads(msg)
        except Exception:
            logger.warning("unparseable command payload: %r", msg)
            return
        if "status" in env:
            return  # a driver's command *reply* (ours included), not a command
        src = env.get("source_type")
        if src == "sim_tele" and not self.accept_sim_tele:
            return  # a simulator's job, not ours
        if src not in ("tele", "sim_tele"):
            return  # edit/edge traffic is not for the aircraft
        cmd = env.get("command")
        if cmd in CONTINUOUS:
            data = env.get("data") or {}
            # SDK bursts carry magnitude in linear_x / angular_z (DJI contract
            # example payload); direction comes from the command name.
            speed = abs(float(data.get("linear_x", data.get("speed", DEFAULT_SPEED))))
            yaw = abs(float(data.get("angular_z", data.get("yaw_rate", DEFAULT_YAW_RATE))))
            ux, uy, uz, ur = CONTINUOUS[cmd]
            with self._cont_lock:
                self._cont_vec = (ux * speed, uy * speed, uz * speed, ur * yaw)
                self._cont_at = time.time()
        else:
            self._cmd_queue.put(env)

    # ------------------------------------------------------------------
    # Threads
    # ------------------------------------------------------------------

    def _pump_loop(self) -> None:
        # Publish the newest cached pose on a fixed deadline: skipping
        # jittery samples instead cost us ~3 of the 10 Hz.
        period = 1.0 / TELEMETRY_HZ
        next_pub = time.time()
        while not self._stop.is_set():
            self.vehicle.pump_once(timeout=0.1)
            now = time.time()
            self._publish_vehicle_state(now)
            if now < next_pub:
                continue
            # advance the grid, never to now, or a stalled loop fires twice
            next_pub += period
            if next_pub < now:
                next_pub = now + period
            pos = self.vehicle.position_enu()
            if pos is not None:
                self._mq.publish_twin_position(self.twin_uuid, *pos)
            quat = self.vehicle.attitude_quat_enu()
            if quat is not None:
                self._mq.update_twin_rotation(self.twin_uuid, quat)
            self._publish_props(now)

    def _publish_vehicle_state(self, now: float, force: bool = False) -> None:
        """Armed flag and flight mode, on change and at least once a second."""
        s = self.vehicle.state
        snapshot = (bool(s["armed"]), s["mode_name"])
        if not force and snapshot == self._state_last \
                and now - self._state_at < STATE_PERIOD_S:
            return
        self._state_last, self._state_at = snapshot, now
        payload = {
            "type": "vehicle_state",
            "armed": snapshot[0],
            "mode": snapshot[1],
            "timestamp": now,
        }
        pwm = s.get("servo_pwm")
        if pwm is not None:
            payload["motors_pwm"] = list(pwm)
        try:
            self._mq.publish(
                f"cyberwave/twin/{self.twin_uuid}/telemetry", payload)
        except Exception:
            logger.debug("vehicle_state publish failed", exc_info=True)

    def _publish_props(self, now: float) -> None:
        """Spin the twin's prop joints from real motor output."""
        pwm = self.vehicle.state.get("servo_pwm")
        if pwm is None:
            return
        dt = min(now - self._prop_last, 0.5)
        self._prop_last = now
        positions, velocities = {}, {}
        for i, name in enumerate(PROP_JOINTS):
            omega = 0.0
            # sanity-bounded: unused channels report 0, and a raw 65535
            # (UINT16 "unknown") must not read as a full-speed prop
            if pwm[i] and 1050 < pwm[i] <= 2200:
                omega = PROP_DIRS[i] * PROP_VISUAL_MAX_RAD_S * \
                    min((pwm[i] - 1000) / 1000.0, 1.0)
            self._prop_angles[i] = (self._prop_angles[i] + omega * dt) % (2 * math.pi)
            positions[name] = self._prop_angles[i]
            velocities[name] = omega
        try:
            # aggregated form: not client-rate-limited, one message for all four
            self._mq.update_joints_state(self.twin_uuid, positions,
                                         velocities=velocities, timestamp=now)
        except Exception:
            logger.debug("prop joint publish failed", exc_info=True)

    def _command_loop(self) -> None:
        while not self._stop.is_set():
            try:
                env = self._cmd_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            cmd, data = env.get("command"), env.get("data") or {}
            logger.info("executing %s %s", cmd, data or "")
            # Contract: discrete commands shut down stick input before executing
            # ("Discrete takeoff, land, RTH, and service commands shut down
            # Virtual Stick before execution") — otherwise the streamer fights
            # the mode change for up to 500 ms.
            with self._cont_lock:
                self._cont_vec = None
            reason, ok = "", False
            try:
                if cmd == "takeoff":
                    ok = self.vehicle.takeoff(float(data.get("altitude", DEFAULT_TAKEOFF_ALT)))
                elif cmd == "land":
                    ok = self.vehicle.land()
                elif cmd == "return_to_home":
                    ok = self.vehicle.return_to_home()
                elif cmd == "stop":
                    self.vehicle.send_velocity_body(0, 0, 0, 0)
                    ok = True
                elif cmd == "emergency_stop":
                    # verified, not assumed: ok only when the armed bit
                    # actually drops (the pump reads it off the heartbeat)
                    ok = self.vehicle.emergency_disarm()
                elif cmd in ("arm", "disarm"):
                    ok, reason = self.vehicle.set_armed(
                        cmd == "arm", force=bool(data.get("force", False)),
                        timeout=ARM_CONFIRM_S)
                else:
                    logger.info("command %r not implemented", cmd)
                    ok = False
                    reason = f"command {cmd!r} not implemented"
            except Exception as exc:
                logger.exception("command %s failed", cmd)
                ok = False
                reason = f"{type(exc).__name__}: {exc}"
            self._reply_status(cmd, ok, reason=reason)
            self._publish_vehicle_state(time.time(), force=True)

    def _reply_status(self, cmd: Optional[str], ok: bool,
                      reason: str = "") -> None:
        """Answer on the command topic (contract direction "both"). Discrete
        commands only — acking 10 Hz continuous bursts would flood the topic.
        `status` is the contract's field; the rest is additive."""
        s = self.vehicle.state
        payload = {
            "status": "ok" if ok else "error",
            "ok": bool(ok),
            "command": cmd,
            "armed": bool(s["armed"]),
            "mode": s["mode_name"],
            "reason": reason,
        }
        pwm = s.get("servo_pwm")
        if pwm is not None:
            payload["motors_pwm"] = list(pwm)
        try:
            self._mq.publish_command_message(self.twin_uuid, payload)
        except Exception:
            logger.warning("could not publish status reply for %s", cmd)

    def _streamer_loop(self) -> None:
        zeroed = True
        while not self._stop.is_set():
            with self._cont_lock:
                vec, at = self._cont_vec, self._cont_at
            if vec is not None and time.time() - at < CONTINUOUS_TIMEOUT_S:
                self.vehicle.send_velocity_body(*vec)
                zeroed = False
            elif not zeroed:
                self.vehicle.send_velocity_body(0, 0, 0, 0)  # dead-man brake
                logger.info("continuous command expired — sticks zeroed")
                zeroed = True
            time.sleep(0.1)

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _mqtt_up(self) -> bool:
        return bool(getattr(self._mq, "connected", False))

    def _connect_mqtt(self) -> None:
        """Connect + subscribe, tolerating failure — run() retries us.
        (A DNS blip at boot on the Pi, 2026-08-30, proved a single
        attempt is not enough on an edge box.)"""
        try:
            self._mq.connect()
            time.sleep(1.0)
            self._subscribe_commands()
        except Exception:
            logger.warning("MQTT connect failed — will retry")

    def run(self) -> None:
        self.vehicle.connect()
        self._connect_mqtt()
        logger.info("driver up: twin=%s aircraft=%s accept_sim_tele=%s",
                    self.twin_uuid, self.vehicle.connection_string, self.accept_sim_tele)

        threads = [
            threading.Thread(target=self._pump_loop, name="pump", daemon=True),
            threading.Thread(target=self._command_loop, name="commands", daemon=True),
            threading.Thread(target=self._streamer_loop, name="streamer", daemon=True),
        ]
        for t in threads:
            t.start()
        try:
            while True:
                time.sleep(10)
                if not self._mqtt_up():
                    logger.warning("MQTT down — reconnecting")
                    self._connect_mqtt()
                s = self.vehicle.state
                logger.info("hb: armed=%s alt=%.2f mode=%s",
                            s["armed"], s["alt"], s["mode_name"])
        except KeyboardInterrupt:
            logger.info("shutting down")
        finally:
            self._stop.set()
            try:
                # best-effort stick zero on the way out (dead-man philosophy:
                # a dying driver must not leave a velocity standing)
                self.vehicle.send_velocity_body(0, 0, 0, 0)
            except Exception:
                pass
            self.vehicle.disconnect()
