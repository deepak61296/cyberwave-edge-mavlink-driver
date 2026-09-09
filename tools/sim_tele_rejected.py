"""B3 negative test: with CYBERWAVE_ACCEPT_SIM_TELE unset, a sim_tele takeoff
must be IGNORED by the driver (contract: only tele executes on the aircraft).

Run while the drone is disarmed on the ground. PASS = driver log shows no
'executing takeoff' after this publishes, aircraft stays disarmed.
"""

import time

from cyberwave import Cyberwave

TWIN = "de6d4efb-eb86-4ca2-b515-7eba576b83ea"

cw = Cyberwave()
cw.mqtt.connect()
time.sleep(1)

cw.mqtt.publish(
    f"cyberwave/twin/{TWIN}/command",
    {
        "source_type": "sim_tele",
        "command": "takeoff",
        "data": {"altitude": 2.0},
        "timestamp": time.time(),
    },
)
print("sim_tele takeoff published — driver must IGNORE it", flush=True)
time.sleep(3)
cw.disconnect()
