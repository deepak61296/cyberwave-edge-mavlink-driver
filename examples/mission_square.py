"""Fly a square — using nothing but the Cyberwave Python SDK.

Every command below is a standard Cyberwave drone command (the same
vocabulary the DJI driver speaks). My MAVLink edge driver picks them
up over MQTT and flies a real ArduPilot autopilot with them.
"""

import time

from cyberwave import Cyberwave

TWIN = "de6d4efb-eb86-4ca2-b515-7eba576b83ea"  # "pixie", my drone twin

cw = Cyberwave()               # auth comes from CYBERWAVE_API_KEY in the env
drone = cw.twin(twin_id=TWIN)  # handle to the twin: commands in, telemetry out

# A discrete command: one MQTT message. The driver does the flying —
# switch to GUIDED, arm, climb — and answers {"status": "ok"} when done.
print("takeoff...")
drone.flight.takeoff(altitude=2.0, source_type="tele")
time.sleep(15)  # let it arm and reach 2 m

for side in range(4):
    # Continuous commands work like RC sticks: the SDK streams them at
    # 10 Hz. If the stream ever stops for 500 ms, the driver zeroes the
    # sticks on its own (dead-man failsafe) — safety is in the contract.
    print(f"side {side + 1}: forward 3 s @ 1 m/s")
    drone.move_forward(1.0, duration=3.0, rate_hz=10.0, source_type="tele")
    time.sleep(2)  # burst over -> dead-man brakes the drone to a hover

    print(f"side {side + 1}: turn left ~90 deg")
    drone.turn_left(0.52, duration=3.0, rate_hz=10.0, source_type="tele")
    time.sleep(2)

print("landing...")
drone.flight.land(source_type="tele")

cw.disconnect()  # the driver finishes the landing on its own
