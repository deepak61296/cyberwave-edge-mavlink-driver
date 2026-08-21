import logging
import os
import sys

from cyberwave_edge_mavlink_driver.driver import CyberwaveEdgeMavlinkDriver

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)


def main() -> None:
    required = ["CYBERWAVE_TWIN_UUID", "CYBERWAVE_API_KEY"]
    missing = [v for v in required if not os.environ.get(v)]
    if missing:
        logger.error("Missing required environment variables: %s", missing)
        sys.exit(1)

    driver = CyberwaveEdgeMavlinkDriver(
        twin_uuid=os.environ["CYBERWAVE_TWIN_UUID"],
        api_key=os.environ["CYBERWAVE_API_KEY"],
        # optional; present when launched by Edge Core, absent standalone
        twin_json_file=os.environ.get("CYBERWAVE_TWIN_JSON_FILE"),
        connection=os.environ.get("MAVLINK_CONNECTION"),  # default tcp:127.0.0.1:5760
    )

    try:
        driver.run()
    except Exception:
        logger.exception("Driver crashed — exiting non-zero so Edge Core can restart")
        sys.exit(1)


if __name__ == "__main__":
    main()
