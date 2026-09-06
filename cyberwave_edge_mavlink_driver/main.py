import logging
import os
import sys

from cyberwave_edge_mavlink_driver.driver import MavlinkDriver

logger = logging.getLogger(__name__)


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    missing = [v for v in ("CYBERWAVE_TWIN_UUID", "CYBERWAVE_API_KEY") if not os.environ.get(v)]
    if missing:
        logger.error("missing environment variables: %s", ", ".join(missing))
        sys.exit(1)
    try:
        MavlinkDriver().run()
    except Exception:
        # non-zero so systemd or Edge Core restarts us
        logger.exception("driver crashed")
        sys.exit(1)


if __name__ == "__main__":
    main()
