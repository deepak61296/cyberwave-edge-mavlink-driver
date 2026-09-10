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

from cyberwave.constants import SOURCE_TYPE_SIM_TELE, SOURCE_TYPE_TELE
from cyberwave.driver import (
    COMMAND_SOURCE_TYPES,
    BaseDriver,
    CallbackGroup,
    CommandArg,
    CommandArgs,
    DriverOperationMode,
    ProtocolArgs,
    PublisherArgs,
    TopicSpec,
    accepts_inbound,
)
from cyberwave.manifest.driver_config import (
    JOINT_UPDATE_TOPIC_SLUG,
    TWIN_POSITION_TOPIC_SLUG,
    TWIN_ROTATION_TOPIC_SLUG,
    TWIN_TELEMETRY_TOPIC_SLUG,
)

from . import contract
from .link import MavlinkLink
from .telemetry import PROP_JOINTS, Telemetry, prop_joint_names
from .vehicle import NAN, Refused, pick_vehicle

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
        # The SDK convention (cyberwave/driver/interface/source_type_policy.py)
        # is wider than this: a teleop listener also takes edit, and takes an
        # envelope with no source_type at all. We do not, by default, because
        # this driver is usually wired to an aircraft that is physically in the
        # room. edit is the twin editor, so accepting it lets someone dragging a
        # marker in a browser tab arm and move that aircraft, and an unstamped
        # envelope has no owner we can name in the log afterwards. Set
        # CYBERWAVE_ACCEPT_ALL_TELE=1 on a simulator or a desk rig to get the
        # upstream policy exactly.
        self.accept_all_tele = os.environ.get("CYBERWAVE_ACCEPT_ALL_TELE", "0") == "1"
        # one thing at a time changes the aircraft's mode: a verb or the sticks
        self._lock = threading.Lock()
        # held for a moment around the two sticks and _running, never across
        # anything that talks to the aircraft: it closes the window where a
        # stick checked _running before a verb set it and stored after the
        # verb had already let the sticks go
        self._stick_lock = threading.Lock()
        self._running = None            # the discrete verb that has the aircraft
        self._dropped = False           # a stick was refused while it runs
        self._stick = None              # (vx, vy, vz, yaw_rate) or None
        self._stick_at = 0.0            # when its window started, None until it does
        self._stick_window = contract.STICK_TIMEOUT_S   # how long it stays live
        self._sticks_live = False
        self._sticks_ready = threading.Event()          # the backend has the aircraft
        # the camera's own stick, with a dead-man of its own so a gimbal move
        # and a flight stick do not have to take turns
        self._gimbal = None             # (pitch_dps, yaw_dps) or None
        self._gimbal_at = 0.0
        self._gimbal_window = contract.STICK_TIMEOUT_S
        self._gimbal_live = False
        self._no_gimbal = False         # this vehicle has none; say so once
        self._streams_at = 0.0
        self._link_up = True            # last state we raised an alert about
        self._pump = None
        self._pump_stop = threading.Event()
        self.stopping = threading.Event()   # the shutdown has begun
        self.ticked_at = None               # last on_tick, for the stall watchdog
        super().__init__(params, twin=twin, **kwargs)

    @classmethod
    def create(cls):
        return cls()

    # -- interface -------------------------------------------------------

    def define_interface(self, iface):
        sources = ProtocolArgs(source_types=["tele", "sim_tele"])
        for name in contract.DISCRETE:
            iface.add_listener(COMMAND_TOPIC, CallbackGroup(self._on_command),
                               protocol=sources, command=self._catalog_entry(name))
        for name in contract.STICK_VERBS:
            iface.add_listener(COMMAND_TOPIC, CallbackGroup(self._on_stick), protocol=sources,
                               command=self._catalog_entry(name, continuous=True, rate_hz=10))
        t = self.telemetry
        self._publish(iface, TWIN_POSITION_TOPIC_SLUG, "TwinPositionPayload", t.position)
        self._publish(iface, TWIN_ROTATION_TOPIC_SLUG, "TwinRotationPayload", t.rotation)
        self._publish(iface, JOINT_UPDATE_TOPIC_SLUG, "JointStatesPayload", t.prop_joints)
        self._publish(iface, TWIN_TELEMETRY_TOPIC_SLUG, "TwinTelemetryPayload", t.vehicle_state)

    @staticmethod
    def _catalog_entry(name, **kwargs):
        """One command as the catalog sees it, arguments and description.

        The contract's table is the source; the SDK turns it into
        commands.specs, which is what the platform and the MCP read.
        """
        description, args = contract.CATALOG[name]
        return CommandArgs(name=name, description=description,
                           args=tuple(CommandArg(*a) for a in args), **kwargs)

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
        # the twin is bound by now, so this is where its joints are readable
        self.telemetry.use_prop_joints(self._prop_joints())
        logger.info("prop joints: %s", ", ".join(self.telemetry.props.joints))

    def _prop_joints(self):
        """The prop joint names this twin's asset actually uses.

        px4vision calls them prop_1_joint..prop_4_joint, the DJI assets
        prop_front_left_joint and so on, so ask the twin instead of
        guessing. CYBERWAVE_PROP_JOINTS names them by hand, for a twin
        that keeps its joints to itself.
        """
        override = os.environ.get("CYBERWAVE_PROP_JOINTS", "").strip()
        if override:
            return [n.strip() for n in override.split(",") if n.strip()]
        try:
            names = prop_joint_names(self.twin.get_controllable_joint_names())
        except Exception:
            names = ()
        if names:
            return names
        logger.info("no prop joints from the twin; using the px4vision names")
        return PROP_JOINTS

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
        logger.info("driver up: aircraft=%s accept_sim_tele=%s accept_all_tele=%s",
                    self.link.connection_string, self.accept_sim_tele,
                    self.accept_all_tele)
        if self.accept_all_tele:
            logger.warning("accepting edit and unstamped commands: simulator rigs only")

    async def on_shutdown(self):
        self.stopping.set()
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
        # paho's connect is a blocking socket call and then up to ten seconds
        # of sleeps waiting for CONNACK; on the loop that is a driver with no
        # ticks, so no stick zeros, no pose and no link watch while it runs
        await asyncio.to_thread(mqtt.connect)
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
        self.ticked_at = now
        self.vehicle.tick()
        self._watch_link()
        if not self._running:   # a discrete verb has the aircraft until it is done
            self._tick_sticks(now)
        # a cold-booted autopilot can miss the first stream request
        if now - self.link.state["last_attitude"] > STREAM_RETRY_S \
                and now - self._streams_at > STREAM_RETRY_S:
            self._streams_at = now
            self.link.request_streams()
        if not self.client.mqtt.connected:
            self._connection_lost.set()

    def _watch_link(self):
        """One alert when the aircraft stops talking, one when it is back."""
        up = self.link.connected()
        if up == self._link_up:
            return
        self._link_up = up
        if up:
            logger.info("mavlink link back after %d reopen(s)", self.link.reopens)
            self._link_alert("MAVLink link back",
                             "The autopilot is sending heartbeats again.", "info",
                             auto_resolve_after=60.0)
        else:
            logger.warning("mavlink link lost")
            self._link_alert("MAVLink link lost",
                             "No heartbeat from the autopilot; the aircraft "
                             "takes no commands until it is back.", "error")

    def _link_alert(self, name, description, severity, **kwargs):
        """Tell the twin. Called from the tick, so the SDK hands the REST
        request to a thread of its own and we never wait on it here."""
        try:
            self.create_twin_alert(name, description=description,
                                   alert_type="mavlink_link", severity=severity,
                                   metadata={"connection": self.link.connection_string,
                                             "reopens": self.link.reopens},
                                   **kwargs)
        except Exception:
            logger.warning("could not raise the link alert")

    def _tick_gimbal(self, now):
        """Keep the camera turning while its stick is fresh; stop it once
        the bursts have stopped coming."""
        rates = self._gimbal
        if rates is not None and now - self._gimbal_at < self._gimbal_window:
            self._gimbal_live = True
            self._drive_gimbal(*rates)
        elif self._gimbal_live:
            self._gimbal_live = False
            self._gimbal = None
            self._drive_gimbal(0.0, 0.0)
            logger.info("gimbal released")

    def _drive_gimbal(self, pitch_dps, yaw_dps):
        """One gimbal rate, from the tick, where nothing may raise."""
        try:
            self.vehicle.gimbal_rate(pitch_dps, yaw_dps)
        except Refused as exc:
            self._gimbal, self._gimbal_live = None, False
            if not self._no_gimbal:
                self._no_gimbal = True
                logger.info("gimbal stick ignored: %s", exc)

    def _tick_sticks(self, now):
        """Stream a fresh stick; release once it has expired."""
        self._tick_gimbal(now)
        stick = self._stick
        if stick is not None and (self._stick_at is None
                                  or now - self._stick_at < self._stick_window):
            self.vehicle.send_velocity_body(*stick)
            if not self._sticks_live:
                self._sticks_live = True
                self._sticks_ready.clear()
                self._off_tick(self.vehicle.prepare_sticks, done=self._sticks_ready)
            if self._stick_at is None and self._sticks_ready.is_set():
                # a distance is flying time, and PX4 spends the first second and
                # a half of a burst entering OFFBOARD: the clock starts here, on
                # the first setpoint the aircraft is really following
                self._stick_at = now
        elif self._sticks_live:
            self._sticks_live = False
            self._stick = None
            self._off_tick(self.vehicle.release_sticks)
            logger.info("sticks released")

    # -- commands --------------------------------------------------------

    def _accepts(self, envelope):
        """tele always; sim_tele and the wider policy opt in; replies never.

        accepts_inbound is the SDK's, so its edge* self-echo guard is the same
        guard every other driver uses. What we narrow is the set we hand it.
        """
        if "status" in envelope:
            return False
        source = envelope.get("source_type")
        if self.accept_all_tele:
            return accepts_inbound(COMMAND_SOURCE_TYPES, source)
        if source is None:
            return False        # the SDK is lenient here; on an aircraft we are not
        allowed = {SOURCE_TYPE_TELE}
        if self.accept_sim_tele:
            allowed.add(SOURCE_TYPE_SIM_TELE)
        return accepts_inbound(frozenset(allowed), source)

    async def _on_command(self, envelope):
        if not self._accepts(envelope):
            return
        if envelope.get("command") in contract.URGENT:
            # here and not in _run: every queued verb holds a worker while it
            # waits for the lock, and a kill behind a full pool would have to
            # wait for one before it could even say it was coming
            self.vehicle.abort.set()
        await asyncio.to_thread(self._run, envelope)

    def _on_stick(self, envelope):
        if not self._accepts(envelope):
            return
        if self._running:
            self._drop_stick()
            return
        cmd = envelope["command"]
        data = envelope.get("data") or {}
        if cmd in contract.GIMBAL_STICKS:
            self._on_gimbal_stick(cmd, data)
            return
        ux, uy, uz, ur = contract.CONTINUOUS[cmd]
        # magnitude rides in the payload, direction comes from the name
        rate = self._magnitude(data, cmd, bool(ur))
        window = self._window(cmd, data, rate)
        if window is None:
            return
        seconds, timed = window
        v = self.vehicle
        if timed and not v.in_air() and not v.armed():
            # a distance is one envelope with a caller waiting on it, and on the
            # ground the sticks move nothing: say so rather than time out silent
            self._reply(cmd, False, "not in air")
            return
        with self._stick_lock:
            if self._running:   # a verb took the aircraft while we read this one
                self._drop_stick()
                return
            self._stick_window = seconds
            self._stick = (ux * rate, uy * rate, uz * rate, ur * rate)
            # a plain stick is a dead-man and runs from the moment it lands; a
            # distance is time on the sticks, so the tick starts its clock instead
            self._stick_at = None if timed else time.time()

    def _drop_stick(self):
        """A burst arriving mid-verb would re-engage GUIDED or OFFBOARD under it."""
        if not self._dropped:
            self._dropped = True
            logger.info("sticks dropped while %s runs", self._running)

    def _on_gimbal_stick(self, cmd, data):
        """A camera stick: the verb names the direction, the payload the rate.

        It moves no aircraft, so unlike a flight stick it is as good on the
        ground as in the air.
        """
        rate = abs(float(data.get("rate", contract.DEFAULT_GIMBAL_RATE)))
        window = self._window(cmd, data, rate)
        if window is None:
            return
        with self._stick_lock:
            if self._running:
                self._drop_stick()
                return
            self._gimbal_window, _ = window
            self._gimbal = (contract.GIMBAL_STICKS[cmd][1] * rate, 0.0)
            self._gimbal_at = time.time()

    @staticmethod
    def _magnitude(data, cmd, turning):
        """The one number a stick carries: a yaw rate for a turn, else a speed.

        The axis the catalog declares for the verb comes first, then the
        generic field the SDK sends whatever the verb.
        """
        generic = ("angular_z", "yaw_rate") if turning else ("linear_x", "speed")
        for name in (contract.STICKS[cmd][1],) + generic:
            if name in data:
                return abs(float(data[name]))
        return contract.DEFAULT_YAW_RATE if turning else contract.DEFAULT_SPEED

    def _window(self, cmd, data, rate):
        """(how long this stick stays live, is that flying time), or None if
        the ask is refused.

        The SDK's flight.ascend(2.0) sends one envelope carrying distance and
        never refreshes it, so the dead-man alone would end the climb after
        half a second. With a distance we stream for as long as it takes to
        fly it; without one nothing changes.
        """
        distance = abs(float(data.get("distance") or 0.0))
        if not distance or not rate:
            return contract.STICK_TIMEOUT_S, False
        travel = distance / rate
        if travel > contract.MAX_TRAVEL_S:
            self._reply(cmd, False, f"too far, more than {contract.MAX_TRAVEL_S:.0f}s of travel")
            return None
        return travel, True

    async def _on_stop_cmd(self, envelope):
        # Not super(): the base answers stop by dropping to NO_OP, which
        # unwires and rewires every subscription. The SDK ends each burst
        # with a stop, and a burst chained straight after could lose
        # envelopes in that gap. Here stop means release the sticks and reply.
        await self._on_command(envelope)

    def _off_tick(self, work, done=None):
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
                finally:
                    if done is not None:
                        done.set()

        threading.Thread(target=run, name="sticks", daemon=True).start()

    def _release_sticks(self):
        # the sticks go under the lock, the aircraft never: a stick that lands
        # between here and the end of the verb finds _running set and is dropped
        with self._stick_lock:
            self._stick = self._gimbal = None
            gimbal_live, self._gimbal_live = self._gimbal_live, False
            sticks_live, self._sticks_live = self._sticks_live, False
        if gimbal_live:
            self._drive_gimbal(0.0, 0.0)
        if sticks_live:
            self.vehicle.release_sticks()
            logger.info("sticks released")

    def _run(self, envelope):
        cmd, data = envelope.get("command"), envelope.get("data") or {}
        with self._lock:    # _on_command has already set abort for an urgent verb
            if cmd in contract.URGENT:
                self.vehicle.abort.clear()
            elif self.vehicle.abort.is_set():
                # an urgent verb is queued behind us; do not make it wait.
                # it releases the sticks itself, so stop still gets its ok.
                self._reply(cmd, cmd == "stop", "" if cmd == "stop" else "superseded")
                return
            logger.info("executing %s %s", cmd, data or "")
            # no handle or no heartbeat: a verb would only wait out its ack.
            # stop is the one verb the contract never refuses.
            if cmd != "stop" and not (self.link.ready() and self.link.connected()):
                self._reply(cmd, False, "not connected")
                return
            with self._stick_lock:
                self._running, self._dropped = cmd, False
            extra = None
            try:
                # the contract: a discrete command shuts stick input down first
                self._release_sticks()
                # a verb may add fields of its own, as takeoff adds altitude_m
                ok, reason, *rest = self._execute(cmd, data)
                extra = rest[0] if rest else None
            except Refused as exc:
                # the vehicle said no in the contract's words; pass them on
                ok, reason = False, str(exc)
            except Exception as exc:
                logger.exception("command %s failed", cmd)
                ok, reason = False, f"{type(exc).__name__}: {exc}"
            finally:
                with self._stick_lock:
                    self._running = None
            if not ok and self.vehicle.abort.is_set():
                reason = "superseded"
            self._reply(cmd, ok, reason, extra)

    def _execute(self, cmd, data):
        v = self.vehicle
        if cmd == "stop":
            return True, ""     # the sticks are already released; that is all stop asks
        if cmd in contract.NEEDS_AIR and not v.in_air() and not v.armed():
            return False, "not in air"
        if cmd == "takeoff":
            if v.in_air():
                return False, "already in air"
            asked = float(data.get("altitude", contract.DEFAULT_TAKEOFF_ALT))
            ok, reason = v.takeoff(asked)
            return ok, reason, {"altitude_m": v.takeoff_altitude(asked)}
        if cmd == "land":
            return v.land()
        if cmd == "return_to_home":
            return v.return_to_home()
        # emergency_stop hovers: the same script must be safe on every
        # aircraft, and kill is the verb that says it cuts the motors
        if cmd in ("brake", "hover", "emergency_stop", "cancel_takeoff",
                   "cancel_landing", "cancel_return_to_home"):
            return v.hold()
        if cmd in ("disarm", "kill") and v.in_air() and not data.get("force", False):
            # motors off in flight drops the aircraft, so it takes a second word
            return False, "in air, send force to override"
        if cmd in ("arm", "disarm"):
            return v.set_armed(cmd == "arm", force=bool(data.get("force", False)))
        if cmd == "kill":
            return v.kill()
        if cmd == "set_home_here":
            return v.set_home_here()
        if cmd == "set_home_location":
            return self._set_home(data)
        if cmd in contract.REBOOT:
            return v.reboot()
        if cmd == "gimbal_rotate":
            return self._gimbal_rotate(data)
        if cmd == "set_gimbal_pitch":
            v.gimbal_point(float(data.get("pitch", 0.0)), NAN, True)
            return True, ""
        if cmd == "gimbal_rotate_speed":
            # the SDK's units are 0.1 deg/s, the vehicle's are deg/s
            v.gimbal_rate(_dps(data.get("pitch")), _dps(data.get("yaw")))
            return True, ""
        if cmd in ("start_compass_calibration", "stop_compass_calibration"):
            start = cmd == "start_compass_calibration"
            if start and v.armed():
                return False, contract.MOTORS_RUNNING
            v.compass_calibration(start)
            return True, ""
        logger.warning("no handler for %s", cmd)
        return False, contract.NOT_SUPPORTED

    def _gimbal_rotate(self, data):
        """Point the camera. An axis the caller left out is not commanded,
        and roll is in the payload but in no gimbal this driver steers."""
        pitch, yaw = _angle(data.get("pitch")), _angle(data.get("yaw"))
        if pitch != pitch and yaw != yaw:
            return False, contract.NOT_SUPPORTED
        absolute = str(data.get("mode", "absolute")).lower() != "relative"
        self.vehicle.gimbal_point(pitch, yaw, absolute, _duration(data))
        return True, ""

    def _set_home(self, data):
        lat, lon = data.get("latitude"), data.get("longitude")
        if lat is None or lon is None:
            return False, "latitude and longitude required"
        lat, lon = float(lat), float(lon)
        if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
            return False, "coordinates out of range"
        altitude = data.get("altitude")
        self.vehicle.set_home(lat, lon, None if altitude is None else float(altitude))
        return True, ""

    def _reply(self, cmd, ok, reason, extra=None):
        """Answer on the command topic. status is the contract's field."""
        payload = {"status": "ok" if ok else "error", "ok": bool(ok), "command": cmd,
                   "reason": reason, "armed": self.vehicle.armed(),
                   "mode": self.vehicle.mode_name(),
                   "flight_state": self.telemetry.flight_state(),
                   "timestamp": time.time()}
        payload.update(extra or {})
        try:
            self.client.mqtt.publish_command_message(self.twin_uuid, payload)
        except Exception:
            logger.warning("could not answer %s", cmd)

    def driver_info_extra(self):
        return self.telemetry.summary()


def _angle(value):
    """One gimbal angle in degrees, NaN for an axis left out."""
    return NAN if value is None else float(value)


def _dps(value):
    """One gimbal rate, from the SDK's 0.1 deg/s to deg/s."""
    return NAN if value is None else float(value) / contract.SPEED_UNIT_PER_DPS


def _duration(data):
    """How long a gimbal move should take. duration is the field the SDK
    documents; the other two turn up in hand-written payloads."""
    for name in ("duration", "duration_sec", "time"):
        if data.get(name) is not None:
            return float(data[name])
    return None
