"""Log every envelope on the twin's command topic — verifies B1 status replies."""

import json
import time

from cyberwave import Cyberwave

TWIN = "de6d4efb-eb86-4ca2-b515-7eba576b83ea"

cw = Cyberwave()
cw.mqtt.connect()


def on_msg(msg):
    env = msg if isinstance(msg, dict) else json.loads(msg)
    kind = "REPLY " if "status" in env else "CMD   "
    print(f"{time.strftime('%H:%M:%S')} {kind} {json.dumps(env)[:200]}", flush=True)


cw.mqtt.subscribe_command_message(TWIN, on_msg)
print("listening on command topic...", flush=True)
while True:
    time.sleep(5)
