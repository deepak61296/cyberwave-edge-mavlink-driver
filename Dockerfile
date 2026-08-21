FROM python:3.11-slim AS base

WORKDIR /app

# Add system dependencies required by your hardware SDK here, e.g.:
# RUN apt-get update && apt-get install -y --no-install-recommends libusb-1.0-0 && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

ENV PYTHONUNBUFFERED=1

ENTRYPOINT ["python", "-m", "cyberwave_edge_mavlink_driver.main"]
