# Base, workdir, the requirements-first layer and the `python -m <pkg>.main`
# entrypoint all come from the driver skill's template
# (cyberwave-os/driver-skill, templates/Dockerfile), which is what scaffolded
# this repo. The SDK itself ships no Dockerfile, so everything below that
# template — the non-root user, the labels, the installed package — is this
# repo's own choice and stays deliberately small.
FROM python:3.11-slim AS base

LABEL org.opencontainers.image.title="cyberwave-edge-mavlink-driver" \
      org.opencontainers.image.description="MAVLink edge driver for ArduPilot and PX4 aircraft" \
      org.opencontainers.image.source="https://github.com/cyberwave-os/cyberwave-edge-mavlink-driver" \
      org.opencontainers.image.licenses="Apache-2.0"

WORKDIR /app

# No system packages: pymavlink and the SDK are pure-python wheels on
# linux/amd64 and linux/arm64. Add apt-get here if that stops being true.

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

# Requirements first so a source edit does not re-resolve the dependencies.
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .
# The package itself, without touching the pinned dependencies above.
RUN pip install --no-cache-dir --no-deps .

# Runs unprivileged. A serial link also needs the group that owns the device
# on the host: `docker run --device /dev/serial0 --group-add dialout`, or the
# numeric gid from `stat -c %g /dev/serial0` when the host names it otherwise.
RUN useradd --create-home --uid 10001 cyberwave && chown -R cyberwave:cyberwave /app
USER cyberwave

# Required, no defaults, supplied at run time:
#   CYBERWAVE_API_KEY    workspace API key
#   CYBERWAVE_TWIN_UUID  the twin this driver drives
# Optional:
#   MAVLINK_CONNECTION   where the autopilot is. Default tcp:127.0.0.1:5760
#                        (SITL on the same host). On a companion computer it
#                        is the mavlink-router slot, udpin:127.0.0.1:14553,
#                        which needs --network host. A direct serial link is
#                        /dev/serial0 with --device and --group-add.
#   CYBERWAVE_REGISTRY_ID       catalog id. Default holybro/px4vision
#   CYBERWAVE_ACCEPT_SIM_TELE   1 lets sim_tele fly it. Bench and SITL only
#   CYBERWAVE_TWIN_JSON_FILE    twin JSON on disk; Edge Core mounts this

ENTRYPOINT ["python", "-m", "cyberwave_edge_mavlink_driver.main"]
