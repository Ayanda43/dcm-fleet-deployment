#!/bin/bash
#
# DCM Fleet Service Installation Helper
# Installs systemd service files for fleet deployment
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
SYSTEMD_DIR="/etc/systemd/system"

echo "=== DCM Fleet Service Installation ==="

# Check for root privileges
if [[ $EUID -ne 0 ]]; then
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

# Install service files
echo "Installing systemd service files..."

for service_file in "$REPO_DIR/systemd"/*.service; do
    if [[ -f "$service_file" ]]; then
        service_name=$(basename "$service_file")
        echo "  Installing: $service_name"
        cp "$service_file" "$SYSTEMD_DIR/"
        chmod 644 "$SYSTEMD_DIR/$service_name"
    fi
done

# Reload systemd daemon
echo "Reloading systemd daemon..."
systemctl daemon-reload

# Enable services
echo "Enabling services..."
systemctl enable zenoh-router.service
systemctl enable rosbridge-websocket.service
systemctl enable cmdr-fleet-ui.service

echo ""
echo "=== Service Installation Complete ==="
echo ""
echo "Services installed:"
echo "  - zenoh-router.service"
echo "  - rosbridge-websocket.service"
echo "  - cmdr-fleet-ui.service"
echo ""
echo "To start services:"
echo "  sudo systemctl start zenoh-router.service"
echo "  sudo systemctl start rosbridge-websocket.service"
echo "  sudo systemctl start cmdr-fleet-ui.service"
echo ""
echo "Or start all at once:"
echo "  sudo systemctl start zenoh-router rosbridge-websocket cmdr-fleet-ui"
