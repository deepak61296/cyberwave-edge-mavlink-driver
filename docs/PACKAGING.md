# Packaging and shipping this driver

Three ways to run it, the catalog file, and how a twin learns the driver
exists. Nothing here needs a flight controller to read.

## The environment it needs

The same set however you run it. Two are required, the rest have defaults.

| Variable | Required | Default | What it is |
|---|---|---|---|
| `CYBERWAVE_API_KEY` | yes | — | Workspace API key. Never bake it into an image |
| `CYBERWAVE_TWIN_UUID` | yes | — | The twin this driver drives |
| `MAVLINK_CONNECTION` | no | `tcp:127.0.0.1:5760` | Where the autopilot is |
| `CYBERWAVE_REGISTRY_ID` | no | `holybro/px4vision` | Catalog asset the twin is registered under |
| `CYBERWAVE_ACCEPT_SIM_TELE` | no | `0` | `1` lets `sim_tele` fly it. Bench and SITL only |

`MAVLINK_CONNECTION` takes any pymavlink connection string. The three that
matter: `tcp:127.0.0.1:5760` for SITL on the same host,
`udpin:127.0.0.1:14553` for a mavlink-router slot on a companion computer,
and `/dev/serial0` for a direct UART to the flight controller.

Only one process can hold the serial port. On a companion computer that
process is mavlink-router, and everything else — this driver, a GCS, a health
monitor — takes a UDP slot behind it.

## 1. systemd on a Pi (what flies today)

This is the shipped path. `provision/` in the parent `cyberwave-drone` repo
does the whole box in one command: builds mavlink-router, clones the driver
into a venv, writes the units, and puts the API key in
`/etc/cyberwave-mavlink-driver.env` root-owned at mode 0600.

```bash
provision/remote-provision.sh --wait          # from the laptop, over Tailscale
```

`provision/README.md` covers the by-hand version and the parameters. What it
ends up with:

- `mavlink-router.service` owns the FC's UART and fans it out. The driver's
  slot is UDP 14553, set by `DRIVER_UDP_PORT`
- `cyberwave-mavlink-driver.service` runs
  `.venv/bin/python -u -m cyberwave_edge_mavlink_driver.main`, with a drop-in
  ordering it after the router
- `Restart=always`, `RestartSec=5`, and the start rate limit turned off. The
  driver exits non-zero on a fatal error on purpose, so with no FC attached
  the unit cycles about every 65 s until one appears. That is the intended
  state, not a fault

`deploy/` in this repo holds a plain unit file and an env template if you want
to install by hand on a box the provisioning kit does not know about.

## 2. Docker

The image is `python:3.11-slim`, runs as uid 10001, and its entrypoint is
`python -m cyberwave_edge_mavlink_driver.main`. Compose is set up for the
companion-computer shape:

```bash
cp .env.example .env       # CYBERWAVE_API_KEY, CYBERWAVE_TWIN_UUID
docker compose up --build
```

`docker-compose.yml` uses `network_mode: host` so `udpin:127.0.0.1:14553`
reaches a mavlink-router already running on the box. A commented block in the
same file swaps that for a serial link.

By hand, against SITL on the host:

```bash
docker build -t cyberwave-edge-mavlink-driver .
docker run --rm --network host \
  -e CYBERWAVE_API_KEY -e CYBERWAVE_TWIN_UUID \
  -e MAVLINK_CONNECTION=tcp:127.0.0.1:5760 \
  cyberwave-edge-mavlink-driver
```

Serial needs the device passed in *and* the group that owns it, because the
container is not root:

```bash
docker run --rm --device /dev/serial0 --group-add "$(stat -c %g /dev/serial0)" \
  -e CYBERWAVE_API_KEY -e CYBERWAVE_TWIN_UUID \
  -e MAVLINK_CONNECTION=/dev/serial0 \
  cyberwave-edge-mavlink-driver
```

Cyberwave's own drivers ship this way — the pixie-cam twin records
`metadata.drivers.default.docker_image = cyberwaveos/camera-driver`, and Edge
Core pulls that image and supervises it. Publishing this one under
`cyberwaveos/` and setting that field on a twin is the step that has not been
done yet.

## 3. A plain venv

For SITL work on a laptop, where a container buys nothing:

```bash
python -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
export CYBERWAVE_API_KEY=... CYBERWAVE_TWIN_UUID=...
python -m cyberwave_edge_mavlink_driver.main
```

`pip install -e .` also works and puts a `driver` console script on the path.

## The catalog file, cw-driver.yml

`cw-driver.yml` at the repo root is the driver's capability contract: every
verb it answers to, the arguments each one takes, and the MQTT topics it
publishes and subscribes. **It is generated. Never edit it by hand.**

It comes from `contract.py` and `define_interface` in `driver.py`, so change
those and then write it again:

```bash
python -m cyberwave_edge_mavlink_driver.main --write-cw-driver
```

Commit the new file with the change that caused it. `test_unit_driver.py`
loads the committed copy and compares it against what the code produces, so a
stale file fails the suite instead of reaching a twin.

Checking it in is a local choice, not an SDK requirement — the SDK is happy to
generate the catalog at startup and never write it down. Having it in git
means a reviewer can see a contract change in the diff.

## How a twin gets the driver registered

Two things have to line up on the twin, and they are separate.

**The contract.** `BaseDriver` posts the catalog to the backend on startup:
`auto_register_interface` is on by default and `REGISTRY_ID` is set, so
`run()` sends the uncompiled `cw-driver.yml` root dict to
`POST /api/v1/twins/{uuid}/driver-schema`. The backend compiles it and writes
the result to the twin's `metadata.mqtt`. From then on the SDK and the MCP can
see what the twin accepts, which is what makes `drone.flight.takeoff(...)`
resolve. Nothing manual is needed; just run the driver once against the twin.

To do it without starting a flight, from a script:

```python
from cyberwave import Cyberwave
from cyberwave_edge_mavlink_driver.driver import MavlinkDriver

twin = Cyberwave().twin(twin_id="<uuid>")
twin.driver.set_schema(MavlinkDriver.get_manifest(compiled=False))
```

The twin must be an asset that can fly. `CYBERWAVE_REGISTRY_ID` has to match
the catalog asset the twin was created from, or the contract lands on the
wrong shape — default `holybro/px4vision`, override it in the env file.

**The image.** Which container runs the twin is recorded separately, at
`metadata.drivers.default.docker_image`. The backend proxies Docker Hub tag
lookups (`/api/v1/docker-registry/tags`) so it can tell an operator a newer
tag exists. Nothing in this repo writes that field yet; the systemd path does
not use it at all.

## What the SDK actually prescribes

Worth knowing, because it is less than you would expect.

`cyberwave-python` ships **no Dockerfile, no compose file and no driver CLI**.
It is a library. The container conventions come from the scaffold this repo
was generated with, `cyberwave-os/driver-skill`, whose
`templates/Dockerfile` is: `python:3.11-slim`, `WORKDIR /app`,
`requirements.txt` installed first, `COPY . .`, `PYTHONUNBUFFERED=1`, and
`ENTRYPOINT ["python", "-m", "<package>.main"]`. The example drivers in the
SDK (`examples/fake_imu_driver.py`, `examples/ursim_driver.py`) are single
files with no packaging at all.

Our Dockerfile keeps all of that and adds three things the template leaves
out: a non-root user, OCI labels, and `pip install .` so the package is
importable from anywhere and not just from `/app`. The non-root user is the
one with a consequence — it is why a serial device needs `--group-add`.

The one convention that is not ours to bend: **a fatal error must exit
non-zero.** Edge Core and systemd both detect a failed start by exit code, and
both restart on it. `main.py` does this deliberately, including a watchdog
that leaves when the tick loop has been silent for a minute.
