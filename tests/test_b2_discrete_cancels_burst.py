"""B2 flight test: a land arriving MID-BURST must cleanly cancel the burst.

Assumes the drone is already airborne (run right after the square lands is
wrong — run it after a fresh takeoff). Sequence: start a 6 s forward burst in
a thread, land 1.5 s into it. PASS = driver log shows land executing with the
streamer silenced (no setpoint/mode fight), aircraft descends.
"""

import threading
import time

from cyberwave import Cyberwave

TWIN = "de6d4efb-eb86-4ca2-b515-7eba576b83ea"

cw = Cyberwave()
d = cw.twin(twin_id=TWIN)

print("takeoff for B2 test...", flush=True)
d.flight.takeoff(altitude=2.0, source_type="tele")
time.sleep(20)  # EKF is warm by now; climb is quick

print("starting 6 s forward burst...", flush=True)
t = threading.Thread(
    target=lambda: d.move_forward(1.0, duration=6.0, rate_hz=10.0, source_type="tele")
)
t.start()
time.sleep(1.5)
print("LAND sent mid-burst (t=1.5s of 6s)", flush=True)
d.flight.land(source_type="tele")
t.join()
print("burst thread finished; check driver log for clean cancel", flush=True)
cw.disconnect()
