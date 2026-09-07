import argparse
import logging
import os
import sys
import threading
import time
from pathlib import Path

from cyberwave_edge_mavlink_driver.driver import MavlinkDriver

logger = logging.getLogger(__name__)

SHUTDOWN_GRACE_S = 4.0   # what the rest of the shutdown gets after ours
CW_DRIVER = Path(__file__).resolve().parents[1] / "cw-driver.yml"
CW_DRIVER_HEADER = ("Generated: python -m cyberwave_edge_mavlink_driver.main "
                    "--write-cw-driver\nEdit contract.py and define_interface, "
                    "then write it again.")


def leave_after_shutdown(driver):
    """End the process once the driver's own shutdown has had its say.

    The last lifecycle alert the SDK sends is an HTTP POST with no request
    timeout, so a read that stalls in it leaves run() blocked for good and
    SIGTERM never gets its process back.
    """
    driver.stopping.wait()
    time.sleep(SHUTDOWN_GRACE_S)
    os._exit(0)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description="MAVLink edge driver for ArduPilot and PX4.")
    parser.add_argument("--write-cw-driver", nargs="?", const=str(CW_DRIVER), metavar="PATH",
                        help="write the cw-driver.yml catalog and exit (default: repo root)")
    args = parser.parse_args()
    if args.write_cw_driver is not None:
        # offline: the catalog comes from define_interface, not from a twin
        MavlinkDriver.get_manifest(path=args.write_cw_driver,
                                   header_comment=CW_DRIVER_HEADER)
        logger.info("wrote %s", args.write_cw_driver)
        return
    missing =[v for v in ("CYBERWAVE_TWIN_UUID", "CYBERWAVE_API_KEY") if not os.environ.get(v)]
    if missing:
        logger.error("missing environment variables: %s", ", ".join(missing))
        sys.exit(1)
    try:
        driver = MavlinkDriver()
        threading.Thread(target=leave_after_shutdown, args=(driver,),
                         name="exit", daemon=True).start()
        driver.run()
    except Exception:
        # non-zero so systemd or Edge Core restarts us
        logger.exception("driver crashed")
        sys.exit(1)


if __name__ == "__main__":
    main()
