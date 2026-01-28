# DCM Fleet Deployment

Automated deployment scripts for DCM (Distributed Control Manager) fleet systems on Revolution Pi hardware.

## Repository Structure

```
dcm-fleet-deployment/
├── deploy-dcm-fleet.sh      # Main deployment script
├── revpi_kernel_setup.sh    # Revolution Pi hardware setup
└── README.md                # This file
```

## Quick Start

1. Clone this repository to the target device:
   ```bash
   sudo mkdir -p /opt/commander/fleet/deployment
   sudo chown developer:developer /opt/commander/fleet
   cd /opt/commander/fleet
   gh repo clone Ayanda43/dcm-fleet-deployment deployment
   ```

2. Run the deployment:
   ```bash
   cd /opt/commander/fleet/deployment
   sudo ./deploy-dcm-fleet.sh
   ```

## Deployment Phases

The deployment script runs through the following phases:

1. **RevPi Setup** - Configures Revolution Pi hardware, kernel modules (requires reboot)
2. **Prerequisites** - Installs git, curl, GitHub CLI
3. **ROS2 Kilted** - Installs ROS2 Kilted with Zenoh middleware
4. **Clone Repository** - Clones `Ayanda43/tsam-dcm` to `/opt/commander/fleet/dcm-control`
5. **Node.js 20.x** - Installs Node.js LTS
6. **Build App** - Runs `npm install && npm run build`
7. **Zenoh Setup** - Runs `scripts/dcm_zenoh_setup.sh` (creates services)
8. **Kiosk Setup** - Configures OpenBox + Chromium kiosk mode

## Installation Paths

| Component | Path |
|-----------|------|
| Fleet Base | `/opt/commander/fleet/` |
| DCM Control | `/opt/commander/fleet/dcm-control/` |
| Deployment | `/opt/commander/fleet/deployment/` |
| Config | `/etc/commander/fleet/` |
| State | `/var/lib/commander/fleet/` |
| Logs | `/var/log/fleet-deployment.log` |

## Services Created

The deployment creates and enables these systemd services (via `dcm_zenoh_setup.sh`):

- `zenoh-router.service` - Zenoh router for ROS2 communication
- `rosbridge-websocket.service` - WebSocket bridge for ROS2 topics
- `app.service` - DCM Control Node.js application

## Verification

After deployment completes:

```bash
# Check service status
systemctl status zenoh-router rosbridge-websocket app

# Test the web interface
curl http://localhost:8090

# View logs
journalctl -u app.service -f
```

## Kiosk Mode

The deployment configures:
- OpenBox window manager
- LightDM auto-login as `developer`
- Chromium in kiosk mode pointing to `http://localhost:8090`

The kiosk starts automatically after reboot.

## Troubleshooting

### Check deployment state
```bash
cat /var/lib/commander/fleet/deployment_state
```

### Resume deployment after failure
```bash
sudo ./deploy-dcm-fleet.sh --continue
```

### View deployment log
```bash
cat /var/log/fleet-deployment.log
```

### Reset deployment state
```bash
sudo rm /var/lib/commander/fleet/deployment_state
sudo ./deploy-dcm-fleet.sh
```

## Requirements

- Ubuntu 24.04 on Raspberry Pi/Revolution Pi
- Internet connection
- GitHub CLI authentication configured (`gh auth login`)
