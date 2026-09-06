# Tests

Two kinds live here.

## Unit tests

`test_unit_link.py`, `test_unit_vehicle.py` and `test_unit_driver.py` need
no aircraft, no broker and no network:

    python -m pytest -q

Bare `pytest` runs only these. `pyproject.toml` pins collection to
`test_unit_*.py` because the scripts below move a real aircraft at import
time.

## Contract-compliance flight tests

Live integration tests against ArduPilot SITL + the Cyberwave broker — not
unit tests. Each proves one constraint of the drone command contract
(`dji/mini-4-pro` schema v1). All three passed on 2026-08-28 against branch
`integration-ardupilot` (evidence: driver + wire-listener logs).

Prereqs: SITL on `tcp:127.0.0.1:5760`, the driver running, and
`CYBERWAVE_API_KEY` exported. Run with the project venv.

| Script | Proves | Pass looks like |
|---|---|---|
| `command_topic_listener.py` | Status replies ride the command topic (direction "both") | `REPLY {"status": "ok", "command": ...}` after each discrete command |
| `test_b2_discrete_cancels_burst.py` | Discrete commands shut down stick input before executing | driver log: `executing land` → `sticks zeroed` within ~30 ms → `mode LAND confirmed`, no mode fight |
| `test_b3_sim_tele_rejected.py` | Only `tele` executes on the aircraft by default | the `sim_tele` takeoff appears on the wire but the driver logs no execution and stays disarmed |

The happy-path test is the mission itself: `../examples/mission_square.py`
(takeoff → 4 continuous-command legs with dead-man hovers → land).
