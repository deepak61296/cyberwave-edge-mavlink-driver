# cyberwave-edge-mavlink-driver

MAVLink edge driver for ArduPilot and PX4 aircraft (SITL and real). Speaks the standard Cyberwave drone command contract.

This driver connects to the [Cyberwave](https://cyberwave.com) platform as a digital twin,
allowing you to monitor and control the device from the Cyberwave dashboard and API.

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/)
- A Cyberwave account — [sign up](https://cyberwave.com)
- Any hardware-specific requirements (cables, SDKs, drivers)

## Local development

```bash
# 1. Install the Cyberwave CLI and log in
pip install cyberwave
cyberwave login

# 2. Create a dev twin and write the .env (replace <registry-id> with your asset)
cyberwave twin create <registry-id> --name "cyberwave-edge-mavlink-driver-dev" --pair --target-dir .

# 3. Create a minimal twin JSON stub
echo '{"metadata": {}}' > /tmp/cyberwave-twin.json

# 4. Build and run
docker compose up --build
```

## Environment variables

| Variable | Required | Description |
|---|---|---|
| `CYBERWAVE_TWIN_UUID` | yes | UUID of the twin instance this driver manages |
| `CYBERWAVE_API_KEY` | yes | API key for authenticating platform calls |
| `CYBERWAVE_TWIN_JSON_FILE` | yes | Path to the writable twin JSON file |
| `CYBERWAVE_CHILD_TWIN_UUIDS` | no | Comma-separated child twin UUIDs |
| `CYBERWAVE_DRIVER_RESTART_LOOP_THRESHOLD` | no | Restart threshold (default: 4) |
| `CYBERWAVE_DRIVER_RESTART_LOOP_WINDOW_SECONDS` | no | Restart window in seconds (default: 60) |

## Deploying to Cyberwave Edge

Push your image to a registry and deploy via the Cyberwave dashboard.
See the [Edge documentation](https://docs.cyberwave.com/edge/drivers/writing-compatible-drivers) for details.

## Reference drivers

- [cyberwave-edge-camera-driver](https://github.com/cyberwave-os/cyberwave-edge-camera-driver)
- [cyberwave-edge-so101](https://github.com/cyberwave-os/cyberwave-edge-so101)

## License

Apache 2.0 — Deepak Popli
