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

| Kind | Commands | MAVLink translation |
|---|---|---|
| discrete | `takeoff`, `land`, `return_to_home`, `stop`, `emergency_stop` | GUIDED+arm+`NAV_TAKEOFF`, LAND, RTL, zero-velocity, force-disarm |
| continuous | `move_forward/backward`, `strafe_left/right`, `turn_left/right`, `ascend`, `descend` | body-frame velocity / yaw-rate setpoints (`SET_POSITION_TARGET_LOCAL_NED`) streamed at 10 Hz |

Contract behaviors honored:

- **Dead-man**: continuous commands must refresh within **500 ms** or the
  driver zeroes the sticks (mirrors the DJI driver and PX4 offboard).
- **`source_type` policy**: `tele` always executes; `sim_tele` only when
  `CYBERWAVE_ACCEPT_SIM_TELE=1` (default on, for SITL development).
- Magnitudes ride in `data.linear_x` / `data.angular_z`; direction comes
  from the command name.

## Architecture

Four loops, one rule: **exactly one thread reads MAVLink**.

- **pump** — sole MAVLink reader; folds messages into shared state and
  publishes position (NED→Z-up) + attitude quaternion to the twin's MQTT
  topics at 10 Hz
- **command worker** — consumes a queue fed by MQTT; discrete commands
  block here (mode verified via HEARTBEAT, actions trusted only on
  COMMAND_ACK), never in the MQTT callback
- **streamer** — emits velocity setpoints while a continuous command is
  fresh; brakes once on expiry
- **MQTT callback** — parse envelope, filter `source_type`, enqueue; never
  blocks

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
- [ ] PX4 SITL pass (OFFBOARD mode + takeoff flow deltas)
- [ ] Battery/status telemetry topics
- [ ] Gimbal commands (contract supports; needs a gimbal target)
- [ ] Real flight controller (bench, props off, then flight — hard safety gates)

Scaffolded with [cyberwave-os/driver-skill](https://github.com/cyberwave-os/driver-skill).
Apache 2.0.
