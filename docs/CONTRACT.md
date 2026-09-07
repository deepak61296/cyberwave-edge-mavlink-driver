# Drone command contract: behaviour specification

Vendor neutral. It says what a driver behind a flying twin must do, refuse and reply, not
which SDK calls it makes, and it is additive: every command already in `commands.supported` keeps its name and
its meaning. Key words: **must**, **should**, **may**.

## 1. Scope

Two topics, for any twin whose asset declares `can_fly: true`.

| Topic | Direction | Carries |
|---|---|---|
| `{prefix}cyberwave/twin/{twin_uuid}/command` | both | command envelopes in, replies out |
| `{prefix}cyberwave/twin/{twin_uuid}/telemetry` | publish | `vehicle_state`, `authority_lost` |

Pose stays on `/position` and `/rotation`; per asset extras such as `/battery/status` and `/gimbal/attitude` are
out of scope. The vocabulary is the 27 commands already published in the drone asset bundle plus five new verbs:
`arm`, `disarm`, `brake`, `hover`, `kill`. Nothing is renamed or removed. Three vendor stacks must fit the same text, and
they differ in ways the spec has to survive.

| Stack | Why it differs |
|---|---|
| DJI MSDK v5 (Android) | The driver is an app beside the remote controller, there is no arming key, flight mode is read only, and the motor cut is a stick gesture the SDK cannot reach, so some verbs have no vendor call at all |
| ArduPilot (MAVLink) | Modes are named, pre-arm checks refuse in the vehicle's own words on a text channel, and a force disarm works in flight |
| PX4 (MAVLink) | Modes are a packed `custom_mode`, takeoff arms after the mode change rather than before, force arming is not reachable over MAVLink, and an acknowledgement alone never proves a verb took effect |

## 2. Envelope and reply

**Command envelope.** Exactly four fields, as built today. A driver must not require a fifth. `data`
may be absent or empty; `timestamp` is epoch seconds.

```json
{"source_type": "tele", "command": "takeoff", "data": {"altitude": 2.0}, "timestamp": 1757000000.0}
```

**Source types.** `source_type` is a routing key, not a label. A driver executes `tele`. It executes `sim_tele`
only if it is a simulator; a driver bound to real hardware ignores `sim_tele` unless explicitly configured to
accept it. It must never execute `edge` or `edge_*`, which is its own output echoing back, and `edit` must
never move a real aircraft.

**Reply.** Published on the same `/command` topic.

```json
{"status": "ok", "ok": true, "command": "arm", "reason": "",
 "armed": true, "mode": "GUIDED", "flight_state": "motors_on", "timestamp": 1757000000.4}
```

`status` is `"ok"` or `"error"`; `ok` is the same fact as a boolean, kept because `status` is the only field the
platform itself names. `reason` is empty on success and a short lowercase phrase on failure: when the vehicle
supplies its own refusal text, pass it through unchanged, otherwise use one of the fixed phrases in section 3.
`armed`, `mode` and `flight_state` report the state at the moment of the reply, not the state requested, and
`mode` is the raw vendor string. Drivers may add fields; two are defined here, `implicit` and
`pending_confirmation`. Three rules follow. Replies are published for discrete commands only, because a continuous
command has the telemetry topic as its evidence. A driver must ignore any envelope on the command topic carrying
a `status` key, including its own replies. One command produces at most one reply.

## 3. Verbs

Reply timing: `ack` means reply once the vehicle has accepted the command, `state` means reply only after the
observable state has changed or a driver timeout has expired, which replies `status: "error"`. Fixed refusal
phrases, for when the vehicle gives no words of its own: `not supported on this vehicle`, `not connected`,
`not in air`, `already in air`, `not armed`, `motors running`, `no position fix`, `no home set`, `nothing to cancel`, `in air, send force to override`.

On the ground with the motors off there is nothing to land, cancel or hold, so `land`, `return_to_home`,
`brake`, `hover`, `emergency_stop` and the three `cancel_*` verbs all refuse `not in air` there. With the
motors running a hold is still a real mode change, so armed on the ground is accepted. A verb the driver has
no handler for refuses `not supported on this vehicle`.

### 3.1 Discrete verbs

| Verb | Data (units) | Must do | Must refuse when | Reply |
|---|---|---|---|---|
| `arm` | `force` bool | Start motors without leaving the ground. On a vehicle with no arming concept, reply ok with `"implicit": true` and do nothing | vehicle refuses, pass its own words | state |
| `disarm` | `force` bool | Stop motors on the ground. No arming concept: ok with `"implicit": true` | in air without `force`: `in air, send force to override` | state |
| `brake` | none | Cancel any automation in progress, zero the sticks, hold position | `not connected` | state |
| `kill` | `force` bool | Cut motors immediately | in air without `force`: `in air, send force to override`. No vendor path: `not supported on this vehicle` | state |
| `emergency_stop` | none | Identical to `brake`. It must not cut motors | as `brake` | state |
| `hover` | none | Identical to `brake`. It is the name the SDK's flight handle sends | as `brake` | state |
| `takeoff` | `altitude` m, a request | Start motors if needed, leave the ground, climb toward `altitude`, hover. Reply carries `altitude_m`, the altitude actually used | `already in air`; vehicle refuses to arm, pass its words | state |
| `land` | none | Descend and land. Where the vehicle has an operator confirm step, reply ok with `"pending_confirmation": true` and hold; a second `land` confirms | `not in air` | state |
| `return_to_home` | none | Fly to the recorded home point and land | `no home set`, `no position fix` | state |
| `cancel_takeoff` | none | Stop climbing, hold here | `nothing to cancel` | state |
| `cancel_landing` | none | Stop descending, hold here | `nothing to cancel` | state |
| `cancel_return_to_home` | none | Stop returning, hold here | `nothing to cancel` | state |
| `stop` | none | Release stick input and hold. Always accepted | never | ack |
| `set_home_here` | none | Record the current position as home | `no position fix` | state |
| `set_home_location` | `latitude`, `longitude` deg, `altitude` m optional | Record the given point as home | missing or out of range coordinates | state |
| `gimbal_rotate` | `pitch`, `yaw`, `roll` deg, `absolute` bool | Rotate the gimbal | `not supported on this vehicle`, or axis not steerable | ack |
| `set_gimbal_pitch` | `pitch` deg, absolute | Set absolute gimbal pitch | as above | ack |
| `gimbal_rotate_speed` | `pitch_rate`, `yaw_rate` deg/s | Rotate at a rate until stopped | as above | ack |
| `start_compass_calibration` | none | Begin calibration | `motors running` | ack |
| `stop_compass_calibration` | none | Abort calibration | never | ack |
| `reboot` | none | Reboot the flight controller | `motors running` | ack, sent before the link drops |
| `reboot_aircraft` | none | Same as `reboot`, two names and one behaviour | same | ack |

`emergency_stop` is pinned to brake because the two shipping drivers disagree about it today: one cancels automation
and hovers, the other force disarms, which on a bench with props off is correct and in flight drops the aircraft. The
same caller script is therefore safe on one vehicle and destructive on the other, which is a defect in the contract
rather than in either driver. `kill` carries the destructive meaning under a name that says so.

### 3.2 Continuous verbs

All continuous verbs are body frame; the verb names the axis and the sign. `data.linear_x` carries translation
speed in m/s and `data.angular_z` carries yaw rate in rad/s, both unsigned magnitudes, defaulting to 1.0 and
0.5 when absent. Gimbal rate verbs take `rate` in deg/s.

| Verb | Axis and sign | Magnitude | Verb | Axis and sign | Magnitude |
|---|---|---|---|---|---|
| `move_forward` | body +X | `linear_x` m/s | `ascend` | up | `linear_x` m/s |
| `move_backward` | body -X | `linear_x` m/s | `descend` | down | `linear_x` m/s |
| `strafe_left` | body -Y | `linear_x` m/s | `turn_left` | yaw counter clockwise | `angular_z` rad/s |
| `strafe_right` | body +Y | `linear_x` m/s | `turn_right` | yaw clockwise | `angular_z` rad/s |
| `gimbal_pitch_up` | gimbal pitch up | `rate` deg/s | `gimbal_pitch_down` | gimbal pitch down | `rate` deg/s |

A continuous command expires 500 ms after the envelope that carried it, and on expiry the driver zeros that input, so callers refresh at 10 Hz to 20 Hz.
Any discrete verb releases stick input before it executes. One that cannot be honoured produces no motion and no reply, and the driver logs it.

## 4. Capability flags

Declared on the asset next to `can_fly`, and readable before anything is sent.

| Flag | Type | Meaning |
|---|---|---|
| `can_arm` | bool | `arm` and `disarm` reach a real vendor call |
| `arm_is_implicit` | bool | motors start as a side effect of `takeoff` |
| `supports_kill` | bool | `kill` reaches a real vendor call |
| `emergency_stop_semantics` | string | `"brake"` |
| `control_frame` | string | `"body"` |

A caller reads them instead of branching on vehicle type:

```python
if drone.capabilities.get("can_arm"):
    drone.arm()
drone.flight.takeoff(altitude=2.0)
```

`arm` replying ok with `implicit: true` on a vehicle that cannot arm is deliberate, so a caller that does not care is not forced to branch.

## 5. flight_state

Published on the telemetry topic beside the fields already sent, as `{"type": "vehicle_state", "flight_state":
"in_air", "vendor_mode": "GUIDED", "armed": true, "timestamp": ...}`. Six values and one legal path:

    disconnected -> ready -> motors_on -> in_air -> returning -> landed -> ready

`landed` returns to `ready` once the motors stop. `returning` is optional and is entered only by `return_to_home`
or a vehicle initiated return. Any state may drop to `disconnected`.

| State | DJI | MAVLink (ArduPilot and PX4) |
|---|---|---|
| `disconnected` | `KeyConnection` false | no vehicle `HEARTBEAT` for 3 s |
| `ready` | connected, `KeyAreMotorsOn` false | heartbeat present, `MAV_MODE_FLAG_SAFETY_ARMED` clear |
| `motors_on` | `KeyAreMotorsOn` true, `KeyIsFlying` false | armed bit set, `EXTENDED_SYS_STATE.landed_state` ON_GROUND |
| `in_air` | `KeyIsFlying` true | `landed_state` IN_AIR, TAKEOFF or LANDING |
| `returning` | `KeyGoHomeStatus` RETURNING_TO_HOME | mode RTL or SMART_RTL (ArduPilot), AUTO.RTL (PX4) |
| `landed` | `KeyIsFlying` false after having been true | `landed_state` ON_GROUND after IN_AIR |

Where `landed_state` is unavailable a driver may fall back to relative altitude above 0.5 m, and must say so in its
driver info. `vendor_mode` is the vehicle's own mode string, published raw and never mapped: `GUIDED`, `AUTO.LOITER`
and `VIRTUAL_STICK` are not the same thing and must not be flattened into one word. Minimum rates: `vehicle_state`
on every change and at least once per second; pose at least 5 Hz while moving.

## 6. Authority and position

A driver publishes an unsolicited event when it loses control authority, with no command in flight and nobody asking:
`{"type": "authority_lost", "reason": "rc_switch", "timestamp": ...}`. `reason` is a short lowercase string. DJI reports
one directly through its authority change reason enum (`RC_LOST`, `RC_SWITCH`, `RC_NOT_P_MODE`, `RC_PAUSE_STOP`,
`RC_ONE_KEY_GO_HOME`, `BATTERY_LOW_GO_HOME`, `NEAR_BOUNDARY` and others). A MAVLink driver emits it when the mode
changes to something it did not command, which is what an RC takeover looks like on that protocol. A caller treats the
event as the end of its own control, not as a warning. The driver also declares which position frame it publishes,
once in its driver info and in every position payload:

| `position_source` | Fields | Units |
|---|---|---|
| `global` | `latitude`, `longitude`, `altitude` | degrees, metres above mean sea level |
| `local` | `x`, `y`, `z` in ENU | metres from the origin the driver declares |

Silence is meaningful. With no fix, or no local origin, the driver publishes nothing on the position topic. It
must not publish zeros, and a caller treats a stale position as unknown rather than as the last known one.

## 7. Conformance

A conformance script drives one twin through the whole vocabulary from the platform side and prints pass or fail per
verb, so two unrelated drivers can be compared without reading either one. For a discrete verb, pass means a single
reply arrived on the command topic inside the driver's timeout, its `command` field matches what was sent, `status` is
`ok`, or `error` with a non empty `reason`, and the `flight_state` in the reply agrees with the telemetry topic within
one second. A refusal is a pass when the refusal is correct: the script sends `kill` without `force` while airborne,
`land` on the ground and `reboot` with motors running, and expects `status: "error"` each time. For a continuous verb,
pass means motion begins within 200 ms, no reply is published, and motion stops within 500 ms of the last envelope.
For `takeoff`, pass additionally means the reply carries `altitude_m`.

Two defects in the published asset definitions limit what a run can prove. The `registry_id` inside
`metadata.mqtt` is doubled (`dji/dji-mini-4-pro` inside an asset whose id is `dji/mini-4-pro`), so anything
resolving a driver by the inner id misses. And `session_id` appears in envelopes in the wild although this
text does not define it and the SDK does not send it.

## 8. Open points

- `arm`, `disarm`, `brake`, `hover` and `kill` are not yet in `commands.supported`, and the SDK refuses anything outside that list before it reaches MQTT.
- Whether `edit` may ever be executed on a live aircraft is undecided. Until it is, a driver treats `edit` as scene editor traffic and drops it.
- There is no known path to set the capability flags of section 4 on a workspace asset, so they stay catalog only for now.
- `vehicle_state` and `authority_lost` have no named payload schemas on the telemetry topic. Section 5 and section 6 are the only definition they have.
- Whether the shipping DJI driver already brakes on `emergency_stop` is unconfirmed. The SDK docstring for it still describes a motor cut, which section 3.1 does not.
- The two defects in section 7 and the four new verbs are independent changes and need not land together.
