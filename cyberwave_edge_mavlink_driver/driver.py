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

Command vocabulary implemented (subset of dji/mini-4-pro contract v1):
  discrete:   takeoff, land, return_to_home, stop, emergency_stop
  continuous: move_forward, move_backward, strafe_left, strafe_right,
              turn_left, turn_right, ascend, descend
"""

import json
import logging
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
DEFAULT_SPEED = 1.0          # m/s for continuous locomotion
DEFAULT_YAW_RATE = 0.5       # rad/s for turns
DEFAULT_TAKEOFF_ALT = 2.0

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
        last_pub = 0.0
        while not self._stop.is_set():
            k = self.vehicle.pump_once(timeout=1.0)
            if k not in ("LOCAL_POSITION_NED", "ATTITUDE"):
                continue
            now = time.time()
            if now - last_pub < 1.0 / TELEMETRY_HZ:
                continue
            last_pub = now
            pos = self.vehicle.position_enu()
            if pos is not None:
                self._mq.publish_twin_position(self.twin_uuid, *pos)
            quat = self.vehicle.attitude_quat_enu()
            if quat is not None:
                self._mq.update_twin_rotation(self.twin_uuid, quat)

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
                    self.vehicle.emergency_disarm()
                    ok = True
                else:
                    logger.info("command %r not implemented", cmd)
                    ok = False
            except Exception:
                logger.exception("command %s failed", cmd)
                ok = False
            self._reply_status(cmd, ok)

    def _reply_status(self, cmd: Optional[str], ok: bool) -> None:
        """Answer on the command topic (contract direction "both"). Discrete
        commands only — acking 10 Hz continuous bursts would flood the topic."""
        try:
            self._mq.publish_command_message(
                self.twin_uuid, {"status": "ok" if ok else "error", "command": cmd})
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

    def run(self) -> None:
        self.vehicle.connect()
        try:
            self._mq.connect()
        except Exception:
            pass
        time.sleep(1.0)
        self._subscribe_commands()
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
                s = self.vehicle.state
                logger.info("hb: armed=%s alt=%.2f mode=%s",
                            s["armed"], s["alt"], s["mode"])
        except KeyboardInterrupt:
            logger.info("shutting down")
        finally:
            self._stop.set()
            self.vehicle.disconnect()
