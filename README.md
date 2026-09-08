# cyberwave-edge-mavlink-driver

Connects any **MAVLink** autopilot, **ArduPilot** or **PX4**, SITL or real
aircraft, to the [Cyberwave](https://cyberwave.com) platform as a live digital
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
braking, land, disarm. All of it mirrored live in the Cyberwave 3D viewer at
10 Hz (position + attitude).

## Command contract

Implements the standard Cyberwave drone vocabulary (as published on the
`dji/mini-4-pro` catalog asset, schema v1):

| Kind | Commands | ArduPilot | PX4 |
|---|---|---|---|
| discrete | `takeoff` | GUIDED, arm, `NAV_TAKEOFF` | `AUTO.TAKEOFF` then arm, confirmed by the landed state |
| discrete | `land`, `return_to_home` | LAND, RTL | `AUTO.LAND`, `AUTO.RTL` |
| discrete | `brake`, `hover`, `emergency_stop`, `cancel_takeoff`, `cancel_landing`, `cancel_return_to_home` | BRAKE (no-op on the ground) | `AUTO.LOITER` |
| discrete | `stop` (registered by the SDK base) | zero the sticks | zero the sticks, then Hold |
| discrete | `kill` | force disarm, refused in the air without `force` | same |
| discrete | `set_home_here`, `reboot`, `reboot_aircraft` | `DO_SET_HOME`, `PREFLIGHT_REBOOT_SHUTDOWN` (refused with the motors running); the two reboot names are one verb | same |
| discrete | `set_home_location` | `DO_SET_HOME` as a `COMMAND_INT`, so the coordinates arrive whole; `altitude` is AMSL, and with none given home keeps the height it has | same; needs a global position |
| discrete | `gimbal_rotate`, `set_gimbal_pitch`, `gimbal_rotate_speed` | `DO_GIMBAL_MANAGER_PITCHYAW`, gimbal protocol v2 | angles the same way; rates through `GIMBAL_MANAGER_SET_ATTITUDE`, since PX4 1.18 ignores the rate fields of the command |
| discrete | `start_compass_calibration`, `stop_compass_calibration` | `DO_START_MAG_CAL`, `DO_CANCEL_MAG_CAL`; the start is refused with the motors running | `PREFLIGHT_CALIBRATION` with the mag flag; the all-zero form cancels; refused with the motors running |
| discrete (extension) | `arm`, `disarm` | `MAV_CMD_COMPONENT_ARM_DISARM`, confirmed against the vehicle's own armed bit; `disarm` is refused in the air without `force` | same; force arm is not possible over MAVLink |
| continuous | `move_forward/backward`, `strafe_left/right`, `turn_left/right`, `ascend`, `descend` | body-frame velocity / yaw-rate setpoints at 10 Hz, in GUIDED | same, in OFFBOARD |
| continuous | `gimbal_pitch_up`, `gimbal_pitch_down` | gimbal pitch rate at 10 Hz, 30 deg/s unless the payload says otherwise | same, as a `GIMBAL_MANAGER_SET_ATTITUDE` rate |

Every discrete command returns `(ok, reason)` from the autopilot backend, so
a refusal comes back with the flight controller's own words. A verb the
aircraft cannot do at all, a gimbal command with no gimbal fitted for one,
answers `not supported on this vehicle`, the contract's phrase for exactly that.

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
driver error. `emergency_stop` cancels automation and hovers; `kill` is the
verb that cuts the motors.

Both wait up to 5 s for the vehicle's `MAV_MODE_FLAG_SAFETY_ARMED` bit to
match and collect `STATUSTEXT` meanwhile, so a refusal comes back in the
flight controller's own words.

### The camera

`gimbal_rotate` carries `pitch`, `yaw` and `roll` in degrees, a `mode` of
`absolute` or `relative`, and an optional `duration`. An axis the caller
leaves out is an axis nobody commanded; `roll` is in the payload the SDK
sends and in no gimbal this driver steers, so a request for roll alone is
refused. `gimbal_rotate_speed` is in the SDK's own units, **0.1 deg/s**, and
the driver converts before the vehicle sees it. A `duration` has no
counterpart in the gimbal protocol, so ArduPilot drives the rate that covers
the gap and then lands on the angle.

ArduPilot takes the pitch/yaw pair or nothing: a command with one half NaN is
refused, so the half the caller left out is filled with the angle the gimbal
already holds. The angle comes back on `GIMBAL_DEVICE_ATTITUDE_STATUS`, or
`MOUNT_STATUS` from a mount too old for the v2 protocol, and goes out in
`vehicle_state` as `gimbal_pitch` and `gimbal_yaw`.

SITL has no gimbal unless it is given one. A servo mount is enough:

```
MNT1_TYPE 1        SERVO9_FUNCTION 7     MNT1_PITCH_MIN -90
MNT1_YAW_MIN -180  SERVO10_FUNCTION 8    MNT1_PITCH_MAX 30
MNT1_YAW_MAX 180
```

Pass them in an extra defaults file (`--defaults copter.parm,gimbal.parm`) so
the mount is there from boot.

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
- `cyberwave/joint/<uuid>/update`: prop spin from real motor PWM, 10 Hz, on
  the prop joint names the twin itself lists
- `cyberwave/twin/<uuid>/telemetry`: `{"type": "vehicle_state", "armed":
  bool, "mode": str, "flight_state": str, "motors_pwm": [...]}`, on every
  change and at least once a second. An aircraft with a gimbal adds
  `gimbal_pitch` and `gimbal_yaw` in degrees, and a change in either
  publishes the record
- twin alerts: one `mavlink_link` alert at severity `error` when the
  autopilot stops sending heartbeats, one at `info` when it is back. Only
  the two transitions, and the REST call runs off the tick loop

The SDK's own `driver_info` snapshot goes out once a second beside these,
and `driver_info_extra()` puts armed, mode and flight_state into it too. That
repeats what `vehicle_state` already carries, deliberately: a consumer
watching the lifecycle snapshot alone still sees the aircraft, while one
watching telemetry gets each change when it happens instead of at the next
second.

Contract behaviors honored:

- **Dead-man**: continuous commands must refresh within **500 ms** or the
  driver zeroes the sticks (mirrors the DJI driver and PX4 offboard). An
  envelope that carries `distance` is the exception: `flight.ascend(2.0)`
  sends one and never refreshes it, so the sticks are held for as long as
  that distance takes at the commanded speed, 30 s at the most. A later
  stick, a `stop` or any hold verb still ends it early.
- **`source_type` policy**: `tele` always executes; `sim_tele` only when
  `CYBERWAVE_ACCEPT_SIM_TELE=1` (default **off**, per the contract: "Only
  source_type tele is executed on the aircraft"; export it for SITL rigs).
  `edit`, `edge`, and untagged envelopes are dropped: stricter than the SDK's
  generic listener policy (which accepts `edit` and untagged), a deliberate
  choice for a flying vehicle.
- Magnitudes ride in `data.linear_x` / `data.angular_z`, or in the axis the
  catalog names for the verb (`linear_y` for a strafe, `linear_z` for a
  climb); direction comes from the command name. Every verb declares its
  arguments and their units, so the catalog and the MCP can see what it takes.

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

`cw-driver.yml` at the repo root is generated, never edited by hand. It comes
from `contract.py` and `define_interface`, so change those and then run
`python -m cyberwave_edge_mavlink_driver.main --write-cw-driver`, committing
the new file with the change that caused it. A unit test compares the
committed catalog against the one the code produces, so a stale file fails
the suite rather than reaching a twin.

`on_reconnect` is the SDK's hook for reopening the device transport, and here
it reopens the broker instead. MAVLink needs no help: the pump thread sees a
socket that has gone silent and rebuilds the link itself. The broker does,
because nothing else notices it, so `on_tick` sets the base's
`_connection_lost` flag as soon as `mqtt.connected` reads false and the base
then calls `on_reconnect` until the client is back.

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

### Environment

| Variable | Default | What it is |
|---|---|---|
| `CYBERWAVE_API_KEY` | — | required |
| `CYBERWAVE_TWIN_UUID` | — | required, the twin to fly |
| `MAVLINK_CONNECTION` | `tcp:127.0.0.1:5760` | serial path, `udpin:host:port`, or SITL TCP |
| `CYBERWAVE_ACCEPT_SIM_TELE` | `0` | also execute `sim_tele` commands; SITL and bench rigs only |
| `CYBERWAVE_REGISTRY_ID` | `holybro/px4vision` | the catalog asset the driver registers as |
| `CYBERWAVE_PROP_JOINTS` | discovered from the twin | prop joints by hand, comma-separated, in spin order |

The prop joints are read from the twin at start: the joint names that
contain `prop`, sorted, up to four. So a DJI asset animates on
`prop_front_left_joint` and a px4vision one on `prop_1_joint`, with no
setting to change. A twin that exposes no joints falls back to
`prop_1_joint..prop_4_joint`, and `CYBERWAVE_PROP_JOINTS` overrides both —
position in that list decides which way each prop turns.

## Status / roadmap

- [x] ArduPilot SITL: full vocabulary proven end-to-end
- [x] PX4 backend written from the in-tree command reference
- [ ] PX4 SITL pass
- [x] Arm / disarm from the SDK, with the FC's refusal reason returned
- [ ] Battery/status telemetry topics
- [x] Gimbal commands on ArduPilot, proven against a SITL servo mount
- [ ] Real flight controller (bench, props off, then flight, hard safety gates)

Scaffolded with [cyberwave-os/driver-skill](https://github.com/cyberwave-os/driver-skill).
Apache 2.0.
