# The contract on DJI MSDK v5

`docs/CONTRACT.md` is written so one vocabulary fits a MAVLink driver and a DJI
driver. This file is the DJI half: the Mobile SDK v5 call behind each verb, for
an Android driver running beside a DJI remote controller. It is documentation
only, there is no DJI code in this repository. Key names are MSDK 5.18.0 and the
aircraft in mind is the Mini 4 Pro.

## Discrete verbs

| Verb | MSDK v5 key or call | Notes |
|---|---|---|
| `arm` | none | v5 has no arming key. Motors start as a side effect of takeoff, so reply ok with `"implicit": true` and do nothing |
| `disarm` | none | Same. ok with `"implicit": true` on the ground, `not supported on this vehicle` in the air, with or without `force` |
| `kill` | none | The in-air motor cut is a stick gesture on the remote controller and is not reachable from the SDK. Always `not supported on this vehicle` |
| `emergency_stop` | zero the sticks, `disableVirtualStick()` | Cancel whatever automation is running and hover. It is not a motor cut, and must not be named after one |
| `brake` | same as `emergency_stop` | Send `KeyStopTakeoff`, `KeyStopAutoLanding` or `KeyStopGoHome` first when one of those is in progress |
| `hover` | same as `brake` | The SDK sends this one already, from the flying twin and from the flight handle, so a DJI driver receives it whether or not the catalog lists it |
| `takeoff` | `KeyStartTakeoff` | Fixed 1.2 m. The `altitude` field is a request, and the reply carries `altitude_m` 1.2. Refused by the aircraft when the motors are already on, which is `already in air`. Climb higher afterwards with `ascend` |
| `cancel_takeoff` | `KeyStopTakeoff` | Stops the climb and hovers at the current altitude |
| `land` | `KeyStartAutoLanding`, then `KeyConfirmLanding` | Below 0.7 m the aircraft parks and raises `KeyIsLandingConfirmationNeeded`. The first `land` replies ok with `"pending_confirmation": true` and raises a Cyberwave alert, a second `land` sends the confirm |
| `cancel_landing` | `KeyStopAutoLanding` | Hovers at the current altitude |
| `return_to_home` | `KeyStartGoHome`, then `KeyGoHomeConfirm(true)` | Same shape as `land`. Some firmwares ask the operator to confirm first, so the driver replies with `"pending_confirmation": true`, raises the alert, and a second `return_to_home` confirms. `KeyGoHomeStatus` carries the progress and gives the `returning` flight state |
| `cancel_return_to_home` | `KeyStopGoHome`, or `KeyGoHomeConfirm(false)` | The confirm key with false is the one to send while the aircraft is parked on the prompt. Once the return is under way it is `KeyStopGoHome`, and the driver picks by state |
| `stop` | zero all four stick axes | Optionally `disableVirtualStick()` afterwards. Always accepted |
| `set_home_here` | `KeyHomeLocationUsingCurrentAircraftLocation` | Refuse with `no position fix` below GPS signal level 4, which is where a home point can be recorded |
| `set_home_location` | `KeyHomeLocation` set | Takes a `LocationCoordinate2D`, so the altitude field is ignored |
| `gimbal_rotate` | `GimbalKey.KeyRotateByAngle` | The SDK sends `mode`, the string `absolute` or `relative`, and an optional `duration` in seconds for a slow cinematic move. Mini 4 Pro gimbal yaw is not independently steerable, so a yaw request is `not supported on this vehicle` |
| `set_gimbal_pitch` | `KeyRotateByAngle`, absolute | |
| `gimbal_rotate_speed` | `KeyRotateBySpeed` | The SDK sends `pitch`, `roll` and `yaw` in 0.1 deg/s, the same unit the key takes, so pass them through. Range -3599 to 3599 |
| `start_compass_calibration` | `KeyStartCompassCalibration` | Refuse with `motors running` while `KeyAreMotorsOn` |
| `stop_compass_calibration` | `KeyStopCompassCalibration` | |
| `reboot`, `reboot_aircraft` | `KeyRebootDevice` | One key for both names. Refuse with `motors running`, which DJI states as a hard rule |

## Continuous verbs

All of them go to `VirtualStickManager.sendVirtualStickAdvancedParam`, after
`enableVirtualStick()` and `setVirtualStickAdvancedModeEnabled(true)`.
`rollPitchCoordinateSystem` is `BODY`, which is what the contract means by body
frame. DJI recommends sending between 5 Hz and 25 Hz, so the contract's 10 Hz to
20 Hz refresh sits inside that. The 500 ms zeroing rule is the driver's own
watchdog and is not a DJI timeout.

| Verb | Field | Control mode |
|---|---|---|
| `move_forward`, `move_backward` | `pitch` | `rollPitchControlMode = VELOCITY` |
| `strafe_left`, `strafe_right` | `roll` | same |
| `ascend`, `descend` | `verticalThrottle` | `verticalControlMode = VELOCITY`, up to 6 m/s. The SDK's flight handle sends these two once with `distance` in metres and no `stop` after it |
| `turn_left`, `turn_right` | `yaw` | `yawControlMode = ANGULAR_VELOCITY` |
| `gimbal_pitch_up`, `gimbal_pitch_down` | `GimbalKey.KeyRotateBySpeed` | 0.1 deg/s units |

Roll and pitch velocity goes up to 23 m/s. Virtual stick is refused within about
30 m of a height or distance limit, and an automatic takeoff interrupts it, so a
driver enables it once the aircraft is airborne and enables it again after any
authority change.

Off-RC teleoperation is opt-in per twin, through
`metadata.drivers.default.virtual_stick = true`. Without that flag the Android
driver rejects every continuous verb with a `failed` status, and the command
still publishes, so the aircraft simply does not move. `failed` is the DJI
driver's word for what the contract calls `error`, and a reader of replies has
to accept both.

## Mode and authority

`KeyFlightMode` is read only, `canSet(false)`. There is no way to command a mode
on DJI, and the contract asks for none: mode is published raw as `vendor_mode`
and never set. Behaviour follows from whichever manager is turned on, and the
mode is the consequence. Virtual stick reads back as `VIRTUAL_STICK`,
`KeyStartGoHome` as `GO_HOME`, `KeyStartAutoLanding` as `AUTO_LANDING`.

Control authority belongs to `RC`, `MSDK` or `OSDK`, and
`FlightControlAuthorityChangeReason` names every way the remote controller takes
it back: `RC_LOST`, `RC_SWITCH`, `RC_NOT_P_MODE`, `RC_PAUSE_STOP`,
`RC_ONE_KEY_GO_HOME`, `BATTERY_LOW_GO_HOME`, `NEAR_BOUNDARY` and others. That
reason, lowercased, is what section 6 of the contract asks a driver to publish as
`authority_lost`. After a grab the driver has to call `enableVirtualStick()`
again before any continuous verb moves the aircraft.

## What only MAVLink can do

- Arm without taking off, and disarm on the ground.
- Cut the motors in flight, with the force parameter.
- Command a flight mode.
- Take off to a chosen altitude instead of a fixed 1.2 m.
- Read and write parameters, and fly to a coordinate under guided control.
- Report per motor output and raw sensor streams.
- Reboot the autopilot at any time, not only with the motors off.
- Run headless on Linux, in Python, against a software simulator.

The contract absorbs this with capability flags and fixed refusals rather than by
cutting verbs. A DJI driver declares `can_arm` false, `arm_is_implicit` true,
`supports_kill` false, and reports in `altitude_m` the altitude it actually used.

## What only DJI has

- An explicit authority model. The SDK says who holds control and why it moved.
  MAVLink has no equivalent, so a MAVLink driver has to infer the same event from
  a mode change it did not command.
- A geofence database. Registration downloads a fly zone database, and
  `KeyHeightLimit`, `KeyDistanceLimit`, `KeyIsNearHeightLimit` and
  `KeyIsNearDistanceLimit` report against it, with virtual stick refused near a
  limit. MAVLink fences are parameters on the airframe with nothing behind them.
- The landing confirmation hold at 0.7 m, which is the reason the contract has
  `pending_confirmation` at all.

## Simulation

The MSDK simulator runs on the flight controller, not on the phone.
`enableSimulator` wraps a flight controller key, so it needs a powered aircraft
on the remote controller link. There is no pure software DJI simulator, and the
arm64 only ABI rules out the Android emulator as well, so nothing on the DJI side
answers to ArduPilot SITL or PX4 SITL.

The shared evidence is therefore the wire, not the vehicle. `tools/conformance.py`
in the project root drives a twin through the vocabulary from the platform side
and prints pass or fail per verb. This driver is judged by it against SITL and a
DJI driver against a real aircraft, and the two runs compare because neither one
reads the driver's own code. For a DJI run the script has to treat a `failed`
reply as `error`, and the twin needs the `virtual_stick` flag set or every
continuous verb fails on purpose.
