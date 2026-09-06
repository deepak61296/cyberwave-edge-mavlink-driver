# cyberwave-edge-mavlink-driver

Connects any **MAVLink** autopilot — **ArduPilot** or **PX4**, SITL or real
aircraft — to the [Cyberwave](https://cyberwave.com) platform as a live digital
twin, speaking the same drone command contract as Cyberwave's DJI driver.

```
ArduPilot / PX4  <-- MAVLink (pymavlink) -->  this driver  <-- MQTT -->  Cyberwave twin
```

With the driver running, plain SDK calls fly the aircraft:

```python
from cyberwave import Cyberwave

cw = Cyberwave()
drone = cw.twin(twin_id="<your-twin-uuid>")
drone.flight.takeoff(altitude=3.0, source_type="tele")
drone.move_forward(1.0, duration=3.0, source_type="tele")
drone.flight.land(source_type="tele")
```

Proven end-to-end against ArduCopter SITL 2026-08-21: takeoff to commanded
altitude, velocity-square via continuous-command bursts with 500 ms dead-man
braking, land, disarm — mirrored live in the Cyberwave 3D viewer at 10 Hz
(position + attitude).

## Command contract

Implements the standard Cyberwave drone vocabulary (as published on the
`dji/mini-4-pro` catalog asset, schema v1):

| Kind | Commands | ArduPilot | PX4 |
|---|---|---|---|
| discrete | `takeoff` | GUIDED, arm, `NAV_TAKEOFF` | `AUTO.TAKEOFF` then arm, confirmed by the landed state |
| discrete | `land`, `return_to_home` | LAND, RTL | `AUTO.LAND`, `AUTO.RTL` |
| discrete | `brake`, `cancel_takeoff`, `cancel_landing`, `cancel_return_to_home` | BRAKE (no-op on the ground) | `AUTO.LOITER` |
| discrete | `stop` | zero the sticks | zero the sticks, then Hold |
| discrete | `emergency_stop`, `kill` | force disarm | force disarm |
| discrete | `set_home_here`, `reboot` | `DO_SET_HOME`, `PREFLIGHT_REBOOT_SHUTDOWN` (refused while armed) | same |
| discrete (extension) | `arm`, `disarm` | `MAV_CMD_COMPONENT_ARM_DISARM`, confirmed against the vehicle's own armed bit | same; force arm is not possible over MAVLink |
| continuous | `move_forward/backward`, `strafe_left/right`, `turn_left/right`, `ascend`, `descend` | body-frame velocity / yaw-rate setpoints at 10 Hz, in GUIDED | same, in OFFBOARD |

Every discrete command returns `(ok, reason)` from the autopilot backend, so
a refusal comes back with the flight controller's own words.

### `arm` / `disarm` (vendor-neutral extension)

A DJI aircraft arms itself on takeoff, so the published contract has no arm
verb. An autopilot does not, hence these two additive commands.

```json
{"source_type": "tele", "command": "arm",    "data": {}}
{"source_type": "tele", "command": "arm",    "data": {"force": true}}
{"source_type": "tele", "command": "disarm", "data": {}}
{"source_type": "tele", "command": "disarm", "data": {"force": true}}
```

`force` sets the `MAV_CMD_COMPONENT_ARM_DISARM` param2 magic number, **2989**
(arm anyway) or **21196** (disarm anyway). Without it param2 is 0 and the
autopilot's own checks decide, so a refusal is a *correct* outcome, not a
driver error. `emergency_stop` still force-disarms.

Both wait up to 5 s for the vehicle's `MAV_MODE_FLAG_SAFETY_ARMED` bit to
match and collect `STATUSTEXT` meanwhile, so a refusal comes back in the
flight controller's own words.

### Status replies

Every discrete command answers on the same command topic (contract
direction "both"). `status` is unchanged; the rest is additive:

```json
{"status": "error", "ok": false, "command": "arm", "reason": "Arm: RC not found",
 "armed": false, "mode": "STABILIZE", "flight_state": "ready", "timestamp": 1757200000.0}
```

`flight_state` is one of `disconnected`, `ready`, `motors_on`, `in_air`,
`returning`, `landed`.

### Twin telemetry

- `cyberwave/twin/<uuid>/position` and `/rotation`: pose, steady 10 Hz
- `cyberwave/joint/<uuid>/update`: prop spin from real motor PWM, 10 Hz
- `cyberwave/twin/<uuid>/telemetry`: `{"type": "vehicle_state", "armed":
  bool, "mode": str, "flight_state": str, "motors_pwm": [...]}`, on every
  change and at least once a second

Contract behaviors honored:

- **Dead-man**: continuous commands must refresh within **500 ms** or the
  driver zeroes the sticks (mirrors the DJI driver and PX4 offboard).
- **`source_type` policy**: `tele` always executes; `sim_tele` only when
  `CYBERWAVE_ACCEPT_SIM_TELE=1` (default **off**, per the contract: "Only
  source_type tele is executed on the aircraft" — export it for SITL rigs).
  `edit`, `edge`, and untagged envelopes are dropped: stricter than the SDK's
  generic listener policy (which accepts `edit` and untagged), a deliberate
  choice for a flying vehicle.
- Magnitudes ride in `data.linear_x` / `data.angular_z`; direction comes
  from the command name.

## Architecture

Built on the SDK's `BaseDriver`, which owns MQTT, the twin, the manifest
and the 10 Hz tick loop. One rule: **exactly one thread reads MAVLink**.

```
cyberwave_edge_mavlink_driver/
  main.py        env check, logging, MavlinkDriver().run()
  driver.py      MavlinkDriver(BaseDriver): commands, sticks, lifecycle
  contract.py    the verb lists, the stick table, flight_state()
  link.py        MavlinkLink: socket, pump, state cache, acks, STATUSTEXT
  vehicle.py     Vehicle: verbs both autopilots share, pick_vehicle(link)
  ardupilot.py   ArduPilot: named modes, GUIDED takeoff, BRAKE hold
  px4.py         PX4: custom_mode main/sub, AUTO.TAKEOFF then arm, Hold
  telemetry.py   what the twin is told: pose, prop spin, vehicle_state
```

- **pump** (thread) is the sole MAVLink reader; it folds messages into the
  link's state cache. Armed and mode come **only** from heartbeats that
  pass `is_vehicle_heartbeat()`; a shared link carries other heartbeats
  and folding one in makes `armed` lie.
- **discrete commands** run one at a time in a worker thread, release the
  sticks first, and answer on the command topic when done.
- **sticks** are stored by the command handler and streamed from the tick
  loop while fresh; on expiry the backend releases them once.
- **publishers** are registry publishers on the tick loop, `source_type`
  `edge`, and run whether or not a controller is attached.
- The backend is picked from `HEARTBEAT.autopilot` (3 ArduPilot, 12 PX4).

## Run it (SITL quickstart)

```bash
# 1. ArduCopter SITL (any recent ArduPilot checkout)
cd <ardupilot>/  && Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy -w

# 2. The driver
pip install -r requirements.txt
export CYBERWAVE_API_KEY=<your key>
export CYBERWAVE_TWIN_UUID=<twin uuid of a can_fly asset>
export MAVLINK_CONNECTION=tcp:127.0.0.1:5760   # default
python -m cyberwave_edge_mavlink_driver.main
```

Watch the twin in the Cyberwave viewer (LIVE tab) and fly it with the SDK
snippet above.

## Status / roadmap

- [x] ArduPilot SITL: full vocabulary proven end-to-end
- [x] PX4 backend written from the in-tree command reference
- [ ] PX4 SITL pass
- [x] Arm / disarm from the SDK, with the FC's refusal reason returned
- [ ] Battery/status telemetry topics
- [ ] Gimbal commands (contract supports; needs a gimbal target)
- [ ] Real flight controller (bench, props off, then flight — hard safety gates)

Scaffolded with [cyberwave-os/driver-skill](https://github.com/cyberwave-os/driver-skill).
Apache 2.0.
