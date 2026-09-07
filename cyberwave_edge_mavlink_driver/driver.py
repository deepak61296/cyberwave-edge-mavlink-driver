"""MAVLink edge driver: one twin, one autopilot.

BaseDriver owns MQTT, the twin and the tick loop. This class turns the
twin's command vocabulary into autopilot verbs and reports the aircraft
back as pose, prop spin and a small vehicle_state record.
"""

import asyncio
import logging
import os
import threading
import time

from cyberwave.driver import (
    BaseDriver,
    CallbackGroup,
    CommandArgs,
    DriverOperationMode,
    ProtocolArgs,
    PublisherArgs,
    TopicSpec,
)
from cyberwave.manifest.driver_config import (
    JOINT_UPDATE_TOPIC_SLUG,
    TWIN_POSITION_TOPIC_SLUG,
    TWIN_ROTATION_TOPIC_SLUG,
    TWIN_TELEMETRY_TOPIC_SLUG,
)

from . import contract
from .link import MavlinkLink
from .telemetry import Telemetry
from .vehicle import pick_vehicle

logger = logging.getLogger(__name__)

STREAM_RETRY_S = 5.0    # ask for streams again when ATTITUDE goes quiet

COMMAND_TOPIC = TopicSpec(namespace="twin", leaf="command",
                          payload_schema_ref="TwinCommandPayload")
ALL_MODES = frozenset(DriverOperationMode)


class MavlinkDriver(BaseDriver):
    """Flies an ArduPilot or PX4 aircraft as a Cyberwave twin."""

    REGISTRY_ID = os.environ.get("CYBERWAVE_REGISTRY_ID", "holybro/px4vision")
    driver_family = "python"
    TICK_RATE_HZ = 10.0
    RECONNECT_MAX_ATTEMPTS = 1_000_000

    def __init__(self, params=None, *, twin=None, **kwargs):
        self.link = MavlinkLink(os.environ.get("MAVLINK_CONNECTION", "tcp:127.0.0.1:5760"))
        self.vehicle = None
        self.telemetry = Telemetry(self.link)
        # Only tele flies the aircraft. sim_tele is opt-in for SITL rigs.
        self.accept_sim_tele = os.environ.get("CYBERWAVE_ACCEPT_SIM_TELE", "0") == "1"
        # one thing at a time changes the aircraft's mode: a verb or the sticks
        self._lock = threading.Lock()
        self._stick = None              # (vx, vy, vz, yaw_rate) or None
        self._stick_at = 0.0
        self._sticks_live = False
        self._streams_at = 0.0
        self._pump = None
        self._pump_stop = threading.Event()
        super().__init__(params, twin=twin, **kwargs)

    @classmethod
    def create(cls):
        return cls()

    # -- interface -------------------------------------------------------

    def define_interface(self, iface):
        sources = ProtocolArgs(source_types=["tele", "sim_tele"])
        for name in contract.DISCRETE:
            iface.add_listener(COMMAND_TOPIC, CallbackGroup(self._on_command),
                               protocol=sources, command=CommandArgs(name=name))
        for name in contract.CONTINUOUS:
            iface.add_listener(COMMAND_TOPIC, CallbackGroup(self._on_stick), protocol=sources,
                               command=CommandArgs(name=name, continuous=True, rate_hz=10))
        t = self.telemetry
        self._publish(iface, TWIN_POSITION_TOPIC_SLUG, "TwinPositionPayload", t.position)
        self._publish(iface, TWIN_ROTATION_TOPIC_SLUG, "TwinRotationPayload", t.rotation)
        self._publish(iface, JOINT_UPDATE_TOPIC_SLUG, "JointStatesPayload", t.prop_joints)
        self._publish(iface, TWIN_TELEMETRY_TOPIC_SLUG, "TwinTelemetryPayload", t.vehicle_state)

    @staticmethod
    def _publish(iface, slug, schema, callback):
        """A 10 Hz edge publisher that also runs with no controller attached.

        The topic is named by slug, not namespace and leaf: the SDK fills the
        twin uuid into a slug, but for a bare namespace it only knows how to
        do that for command, telemetry and joint/update, so pose would go out
        on a topic with the placeholder still in it.
        """
        iface.add_publisher(TopicSpec(topic_slug=slug, payload_schema_ref=schema),
                            CallbackGroup(callback), protocol=ProtocolArgs(source_types=["edge"]),
                            publisher=PublisherArgs(rate_hz=10), operation_modes=ALL_MODES)

    # -- lifecycle -------------------------------------------------------

    async def on_configure(self):
        pass

    async def on_connect_to_device(self):
        await asyncio.to_thread(self.link.connect)
        self.vehicle = self.telemetry.vehicle = pick_vehicle(self.link)
        logger.info("autopilot: %s", self.vehicle.name)

    async def on_register_callbacks(self):
        pass

    async def on_activate(self):
        self._pump = threading.Thread(target=self._pump_loop, name="pump", daemon=True)
        self._pump.start()
        self._streams_at = time.time()
        self.link.request_streams()
        logger.info("driver up: aircraft=%s accept_sim_tele=%s",
                    self.link.connection_string, self.accept_sim_tele)

    async def on_shutdown(self):
        self._pump_stop.set()
        if self._pump is not None:
            self._pump.join(timeout=2.0)
        try:
            # a dying driver must not leave a velocity standing
            if self.vehicle is not None:
                self.vehicle.send_velocity_body(0, 0, 0, 0)
        except Exception:
            pass
        self.link.close()

    async def on_reconnect(self):
        mqtt = self.client.mqtt
        mqtt.connect()
        for _ in range(100):
            if mqtt.connected:
                return True
            await asyncio.sleep(0.1)
        return False

    def _pump_loop(self):
        """The only MAVLink reader."""
        while not self._pump_stop.is_set():
            try:
                self.link.pump_once(timeout=0.1)
            except Exception:
                logger.exception("mavlink read failed")
                time.sleep(0.5)

    async def on_tick(self):
        now = time.time()
        self.vehicle.tick()
        stick = self._stick
        if stick is not None and now - self._stick_at < contract.STICK_TIMEOUT_S:
            self.vehicle.send_velocity_body(*stick)
            if not self._sticks_live:
                self._sticks_live = True
                self._off_tick(self.vehicle.prepare_sticks)
        elif self._sticks_live:
            self._sticks_live = False
            self._stick = None
            self._off_tick(self.vehicle.release_sticks)
            logger.info("sticks released")
        # a cold-booted autopilot can miss the first stream request
        if now - self.link.state["last_attitude"] > STREAM_RETRY_S \
                and now - self._streams_at > STREAM_RETRY_S:
            self._streams_at = now
            self.link.request_streams()
        if not self.client.mqtt.connected:
            self._connection_lost.set()

    # -- commands --------------------------------------------------------

    def _accepts(self, envelope):
        """tele always; sim_tele only when enabled; replies, ours too, never."""
        if "status" in envelope:
            return False
        source = envelope.get("source_type")
        return source == "tele" or (source == "sim_tele" and self.accept_sim_tele)

    async def _on_command(self, envelope):
        if self._accepts(envelope):
            await asyncio.to_thread(self._run, envelope)

    def _on_stick(self, envelope):
        if not self._accepts(envelope):
            return
        data = envelope.get("data") or {}
        # magnitude rides in the payload, direction comes from the name
        speed = abs(float(data.get("linear_x", data.get("speed", contract.DEFAULT_SPEED))))
        yaw = abs(float(data.get("angular_z", data.get("yaw_rate", contract.DEFAULT_YAW_RATE))))
        ux, uy, uz, ur = contract.CONTINUOUS[envelope["command"]]
        self._stick = (ux * speed, uy * speed, uz * speed, ur * yaw)
        self._stick_at = time.time()

    async def _on_stop_cmd(self, envelope):
        await self._on_command(envelope)
        await super()._on_stop_cmd(envelope)

    def _off_tick(self, work):
        """Run a stick mode change away from the tick.

        Entering OFFBOARD takes seconds and needs the setpoint stream to keep
        running through it; doing it on the tick stops both that stream and
        every publisher, so the twin's pose freezes mid-manoeuvre.
        """
        def run():
            with self._lock:
                try:
                    work()
                except Exception:
                    logger.exception("stick mode change failed")

        threading.Thread(target=run, name="sticks", daemon=True).start()

    def _release_sticks(self):
        self._stick = None
        if self._sticks_live:
            self._sticks_live = False
            self.vehicle.release_sticks()
            logger.info("sticks released")

    def _run(self, envelope):
        cmd, data = envelope.get("command"), envelope.get("data") or {}
        if cmd in contract.URGENT:
            self.vehicle.abort.set()    # whatever is running gives way
        with self._lock:
            if cmd in contract.URGENT:
                self.vehicle.abort.clear()
            elif self.vehicle.abort.is_set():
                # an urgent verb is queued behind us; do not make it wait
                self._reply(cmd, False, "superseded")
                return
            logger.info("executing %s %s", cmd, data or "")
            # the contract: a discrete command shuts stick input down first
            self._release_sticks()
            try:
                ok, reason = self._execute(cmd, data)
            except Exception as exc:
                logger.exception("command %s failed", cmd)
                ok, reason = False, f"{type(exc).__name__}: {exc}"
            if not ok and self.vehicle.abort.is_set():
                reason = "superseded"
            self._reply(cmd, ok, reason)

    def _execute(self, cmd, data):
        v = self.vehicle
        if cmd == "stop":
            return True, ""     # the sticks are already released; that is all stop asks
        if cmd == "takeoff":
            if v.in_air():
                return False, "already in air"
            return v.takeoff(float(data.get("altitude", contract.DEFAULT_TAKEOFF_ALT)))
        if cmd == "land":
            return v.land()
        if cmd == "return_to_home":
            return v.return_to_home()
        # emergency_stop hovers: the same script must be safe on every
        # aircraft, and kill is the verb that says it cuts the motors
        if cmd in ("brake", "emergency_stop", "cancel_takeoff",
                   "cancel_landing", "cancel_return_to_home"):
            return v.hold()
        if cmd in ("arm", "disarm"):
            return v.set_armed(cmd == "arm", force=bool(data.get("force", False)))
        if cmd == "kill":
            # cutting the motors in flight drops the aircraft, so say it out loud
            if v.in_air() and not data.get("force", False):
                return False, "refused: in the air, send force to cut the motors anyway"
            return v.kill()
        if cmd == "set_home_here":
            return v.set_home_here()
        if cmd == "reboot":
            return v.reboot()
        return False, f"command {cmd!r} not implemented"

    def _reply(self, cmd, ok, reason):
        """Answer on the command topic. status is the contract's field."""
        payload = {"status": "ok" if ok else "error", "ok": bool(ok), "command": cmd,
                   "reason": reason, "armed": self.vehicle.armed(),
                   "mode": self.vehicle.mode_name(),
                   "flight_state": self.telemetry.flight_state(),
                   "timestamp": time.time()}
        try:
            self.client.mqtt.publish_command_message(self.twin_uuid, payload)
        except Exception:
            logger.warning("could not answer %s", cmd)

    def driver_info_extra(self):
        return self.telemetry.summary()
