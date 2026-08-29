"""Fly the square by publishing contract envelopes directly via MQTT.

Same wire traffic as mission_square.py, but built by hand — useful when
the twin's capability snapshot hides the SDK's .flight sugar. The driver
neither knows nor cares who built the envelope.
"""

import time

from cyberwave import Cyberwave

TWIN = "de6d4efb-eb86-4ca2-b515-7eba576b83ea"
TOPIC = f"cyberwave/twin/{TWIN}/command"

cw = Cyberwave()
cw.mqtt.connect()
time.sleep(1)


def send(command, data=None):
    cw.mqtt.publish(TOPIC, {
        "source_type": "tele", "command": command,
        "data": data or {}, "timestamp": time.time(),
    })


def burst(command, data, seconds, hz=10.0):
    end = time.time() + seconds
    while time.time() < end:
        send(command, data)
        time.sleep(1.0 / hz)


print("takeoff...")
send("takeoff", {"altitude": 2.0})
time.sleep(15)

for leg in range(4):
    print(f"leg {leg + 1}: forward burst 3 s @ 1 m/s")
    burst("move_forward", {"linear_x": 1.0}, 3.0)
    time.sleep(2)  # dead-man brakes to hover
    print(f"leg {leg + 1}: turn left ~90 deg")
    burst("turn_left", {"angular_z": 0.52}, 3.0)
    time.sleep(2)

print("landing...")
send("land")
print("mission sent - the driver does the rest")
time.sleep(2)
cw.disconnect()
