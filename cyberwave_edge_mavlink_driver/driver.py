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
        self.accept_sim_tele = os.environ.get("CYBERWAVE_ACCEPT_SIM_TELE", "1") != "0"

        conn = connection or os.environ.get("MAVLINK_CONNECTION", "tcp:127.0.0.1:5760")
        self.vehicle = MavlinkVehicle(conn)

        self._cmd_queue: "queue.Queue[dict]" = queue.Queue()
        self._cont_lock = threading.Lock()
        self._cont_vec: Optional[tuple] = None   # (vx, vy, vz, yaw_rate)
        self._cont_at = 0.0
        self._stop = threading.Event()

        self._cw = Cyberwave()
        self._mq = self._cw.mqtt

    # ------------------------------------------------------------------
    # MQTT side
    # ------------------------------------------------------------------

    def _subscribe_commands(self) -> None:
        topic = f"cyberwave/twin/{self.twin_uuid}/command"
        # NOTE: no public command-subscribe helper in SDK v0.6.5 — using the
        # inner client, as proven live in probe_flight.py. SDK PR candidate.
        self._mq._client.subscribe(topic, self._on_command)
        logger.info("subscribed to %s", topic)

    def _on_command(self, msg: Any) -> None:
        """paho thread: validate + enqueue, never block."""
        try:
            env = msg if isinstance(msg, dict) else json.loads(msg)
        except Exception:
            logger.warning("unparseable command payload: %r", msg)
            return
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
            try:
                if cmd == "takeoff":
                    self.vehicle.takeoff(float(data.get("altitude", DEFAULT_TAKEOFF_ALT)))
                elif cmd == "land":
                    self.vehicle.land()
                elif cmd == "return_to_home":
                    self.vehicle.return_to_home()
                elif cmd == "stop":
                    with self._cont_lock:
                        self._cont_vec = None
                    self.vehicle.send_velocity_body(0, 0, 0, 0)
                elif cmd == "emergency_stop":
                    self.vehicle.emergency_disarm()
                else:
                    logger.info("command %r not implemented yet", cmd)
            except Exception:
                logger.exception("command %s failed", cmd)

    def _streamer_loop(self) -> None:
        zeroed = True
        primed = 0
        last_mode_req = 0.0
        while not self._stop.is_set():
            with self._cont_lock:
                vec, at = self._cont_vec, self._cont_at
            fresh = vec is not None and time.time() - at < CONTINUOUS_TIMEOUT_S
            if fresh:
                self.vehicle.send_velocity_body(*vec)
                zeroed = False
                # PX4: OFFBOARD refuses to engage until setpoints are already
                # flowing — prime a few, then request the mode until it sticks.
                if self.vehicle.is_px4 and not self._in_offboard():
                    primed += 1
                    if primed >= 3 and time.time() - last_mode_req > 1.0:
                        self.vehicle.m.set_mode("OFFBOARD")
                        last_mode_req = time.time()
                else:
                    primed = 0
            elif not zeroed:
                self.vehicle.send_velocity_body(0, 0, 0, 0)  # dead-man brake
                logger.info("continuous command expired — sticks zeroed")
                zeroed = True
            elif self.vehicle.is_px4 and self._in_offboard():
                # dropping the stream would trip PX4's offboard failsafe —
                # keep zero-velocity setpoints flowing (= position hold)
                # until a discrete command switches the mode.
                self.vehicle.send_velocity_body(0, 0, 0, 0)
            time.sleep(0.1)

    def _in_offboard(self) -> bool:
        try:
            return self.vehicle.state["mode"] == self.vehicle.m.mode_mapping()["OFFBOARD"]
        except Exception:
            return False

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
