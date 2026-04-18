#!/bin/bash
# install_serial_dds_service.sh
# Builds pi_bridge and installs the systemd service on Raspberry Pi.
# Run with: bash pi_bridge/scripts/install_serial_dds_service.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PI_BRIDGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PI_BRIDGE_DIR/build"
BINARY_NAME="pi_bridge"
SYMLINK_PATH="/usr/local/bin/robot-serial-dds"
SERVICE_NAME="pi-serial-dds"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

# ─── 1. Build ────────────────────────────────────────────────────────────────
echo "[1/4] Building pi_bridge..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -- -j"$(nproc)"
echo "      Build done: $BUILD_DIR/$BINARY_NAME"

# ─── 2. Symlink ──────────────────────────────────────────────────────────────
echo "[2/4] Installing symlink at $SYMLINK_PATH..."
sudo ln -sf "$BUILD_DIR/$BINARY_NAME" "$SYMLINK_PATH"
echo "      $SYMLINK_PATH -> $BUILD_DIR/$BINARY_NAME"

# ─── 3. Write service file ───────────────────────────────────────────────────
echo "[3/4] Writing service file to $SERVICE_FILE..."
sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=DDS Pi - robot UART Communication data Service
Wants=network-online.target
After=network-online.target systemd-udev-settle.service pi-lidar-dds.service

[Service]
Type=simple
User=root
WorkingDirectory=/usr/local/bin

# Wait for /dev/mcu_uart to appear (timeout 15 s)
ExecStartPre=/bin/sh -c 'for i in \$(seq 1 30); do [ -e /dev/mcu_uart ] && exit 0; sleep 0.5; done; exit 1'
# Extra delay so pi-lidar-dds finishes FastDDS init before we create our own participants
ExecStartPre=/bin/sleep 15

ExecStart=$SYMLINK_PATH
Restart=always
RestartSec=2
TimeoutStopSec=20

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF
echo "      Service file written."

# ─── 4. Enable & start ───────────────────────────────────────────────────────
echo "[4/4] Enabling and starting ${SERVICE_NAME}.service..."
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}.service"
sudo systemctl restart "${SERVICE_NAME}.service"

echo ""
echo "Done! Service status:"
sudo systemctl status "${SERVICE_NAME}.service" --no-pager -l
