# DCM Fleet Deployment

Automated deployment system for DCM (Distributed Control Manager) fleet systems on Revolution Pi hardware.

## Table of Contents

- [Overview](#overview)
- [Deployment Architecture](#deployment-architecture)
- [Prerequisites](#prerequisites)
- [Hardware Preparation](#hardware-preparation)
- [Deployment Process](#deployment-process)
- [Post-Deployment](#post-deployment)
- [Troubleshooting](#troubleshooting)
- [Deployment Checklist](#deployment-checklist)
- [Version Information](#version-information)

## Overview

The DCM Fleet deployment system provides automated installation and configuration of the complete DCM software stack onto Revolution Pi Connect 5 hardware. The deployment process installs ROS2 Kilted, Node.js, builds the DCM application, configures Zenoh networking, and sets up a kiosk display for the control interface.

### Deployment Methods

1. **Interactive Deployment** - Primary method with prompts for confirmation
   ```bash
   sudo ./deploy-dcm-fleet.sh
   ```

2. **Automated Deployment** - Non-interactive mode for scripted deployments
   ```bash
   sudo ./deploy-dcm-fleet.sh --auto
   ```

3. **Custom Repository** - Deploy with alternative DCM repositories
   ```bash
   sudo ./deploy-dcm-fleet.sh --dcm=corolla
   sudo ./deploy-dcm-fleet.sh --repo-url=https://github.com/Ayanda43/corrolla-dcm.git
   sudo ./deploy-dcm-fleet.sh --repo-url=Ayanda43/tsam-dcm
   ```

### Target Hardware

- **Platform:** Revolution Pi Connect 5 (ARM64)
- **Operating System:** Ubuntu 24.04 (Noble)
- **Storage:** Minimum 32GB recommended
- **Network:** Ethernet/WiFi with internet access during deployment
- **CAN Interface:** Required for hardware communication

## Deployment Architecture

### Directory Structure

```
/opt/commander/fleet/
├── deployment/                     # This repository
│   ├── deploy-dcm-fleet.sh         # Main deployment script
│   └── revpi_kernel_setup.sh       # RevPi hardware setup script
└── dcm-control/                    # Cloned DCM repository
    ├── scripts/
    │   └── dcm_zenoh_setup.sh      # Zenoh router & services setup
    └── config/zenoh/               # Zenoh configuration files

/etc/commander/fleet/
└── fleet.env                       # Environment configuration

/var/lib/commander/fleet/
└── deployment_state                # State persistence for reboot handling

/var/log/fleet-deployment.log       # Deployment logs
```

### Systemd Services

| Service | Description | User |
|---------|-------------|------|
| `zenoh-router.service` | Zenoh DDS router | root |
| `rosbridge-websocket.service` | ROS2-WebSocket bridge | developer |
| `cmdr-fleet-ui.service` | DCM web interface | developer |
| `lightdm` | Display manager (kiosk) | system |

## Prerequisites

### Hardware Requirements

- Revolution Pi Connect 5 with ARM64 processor
- Minimum 4GB RAM
- 32GB+ storage (eMMC or SD card)
- CAN interface configured
- Network connectivity (Ethernet or WiFi)
- Display connected (for kiosk mode)

### Network Requirements

- Internet access during deployment (for package downloads)
- Access to GitHub (github.com)
- Access to ROS2 package repositories (packages.ros.org)
- Access to NodeSource repository (deb.nodesource.com)

### Required Credentials

- GitHub account with access to DCM repositories
- SSH keys or Personal Access Token for GitHub authentication (for private repos)
- `sudo` privileges on target hardware

### Software Prerequisites (Installed by Script)

The deployment script automatically installs:
- ROS2 Kilted distribution (ros-base)
- ROS2 Zenoh RMW implementation (rmw-zenoh-cpp)
- ROS2 rosbridge server
- GitHub CLI (`gh`)
- Node.js 20.x LTS
- npm 10.x
- Build tools (git, curl, wget, build-essential)
- Kiosk packages (LightDM, OpenBox, Chromium)

## Hardware Preparation

Before running the deployment script, the Revolution Pi hardware must be prepared with Ubuntu Server 24.04 using Raspberry Pi Imager. This is a one-time setup process for each new RevPi unit.

### RevPi Connect 5 to Ubuntu Server 24.04 Setup

#### Required Tools
- USB-C cable with **both power AND data transfer** capability
- Laptop with [Raspberry Pi Imager](https://www.raspberrypi.com/software/) installed
- [usbboot](https://github.com/raspberrypi/usbboot) tool for eMMC recognition

#### Installation Steps

**1. Connect RevPi to Laptop**
- Plug USB-C cable into RevPi and laptop
- Ensure USB-C cable is connected **BEFORE** powering on the RevPi
- Power on the RevPi

**2. Recognize eMMC as Bootable**
- Download and follow instructions at [https://github.com/raspberrypi/usbboot](https://github.com/raspberrypi/usbboot)
- Run `sudo ./rpiboot` from the `usbboot` directory to recognize RevPi eMMC as bootable

**3. Image eMMC with Raspberry Pi Imager**
- Run `Raspberry Pi Imager` application
- Under `CHOOSE DEVICE`, select `Raspberry Pi 5`
- Under `CHOOSE OS`, select `Other general-purpose OS > Ubuntu > Ubuntu Server 24.04... LTS (64-bit)`
- Under `CHOOSE STORAGE`, select `mmcblk0`
- Click `Next`
- Under `Use OS Customization`, select `EDIT SETTINGS`
  - Under `GENERAL`:
    - Set hostname: `<device-identifier>` (e.g., `fleet001`)
    - Set username and password:
      - Username: `developer`
      - Password: `<your-secure-password>`
  - Under `SERVICES`:
    - Enable SSH
    - Use password authentication
  - Click `SAVE`
  - When prompted "Would you like to apply OS customization?": Select `YES`

**4. Complete Imaging**
- Wait for imaging process to complete
- Power off device and remove USB-C cable
- Connect RevPi to ethernet and power on

**5. Initial Network Setup**
- SSH into device: `ssh developer@<hostname>` (or use monitor and keyboard)
- The RevPi kernel setup script will be run by the deployment process

**6. Configure RevPi Hardware via Cockpit**
- Go to Cockpit at `https://<hostname>:41443/`
- Login with developer credentials
- Ensure you have `Administrative access`
- Update the system if needed

**7. Configure PiCtory**
- Under `RevPi Configuration` > `PiCtory`, select `PiCtory`
- Drag and drop `RevPi Connect 5` and other devices to the DIN rail as physically installed
- Save configuration: `File` > `Save as Start-Config`
- Update the system: `Tools` > `Reset Driver`

**8. Network Configuration**
- Setup network connections using: `sudo nmtui`

### Verification Tests

After hardware preparation, verify the RevPi is configured correctly:

**1. Check Kernel**
```bash
uname -a
# Expected output (similar to):
# Linux ubuntu 6.6.0-revpi7-rpi-v8 #1 SMP PREEMPT_RT ... aarch64 GNU/Linux
```

**2. Check RevPi Modules**
```bash
piTest -d
# Expected output (similar to):
# Found 2 devices:
#
# Address: 0 module type: 138 (0x8a) RevPi Connect 5 V1.0
# Module is present
```

**3. Check Network Interfaces**
```bash
ip link show
# Should show:
# - can0 and can1
# - pileft and piright
# - eth0 and eth1
# - wlan0
```

**4. Test LEDs**
```bash
piTest -w RevPiLED,1
piTest -w RevPiLED,2
# LEDs should change colors
```

## Deployment Process

### High-Level Overview

The deployment consists of 8 phases:

```
Phase 1: RevPi Hardware Setup
    ├─> Load kernel modules
    ├─> Configure CAN interface
    ├─> Setup piControl
    └─> Schedule reboot for kernel changes

Phase 2: Prerequisites & Authentication
    ├─> Create deployment directories
    ├─> Verify internet connectivity
    ├─> Install git, curl, wget, build-essential
    └─> Install GitHub CLI

Phase 3: ROS2 Kilted Installation
    ├─> Add ROS2 repository
    ├─> Install ros-kilted-ros-base
    ├─> Install rmw-zenoh-cpp
    ├─> Install rosbridge-server
    └─> Initialize rosdep

Phase 4: Clone Repository
    ├─> Create /opt/commander/fleet directory
    ├─> Clone DCM repository (tsam-dcm by default)
    └─> Or clone custom repo via --dcm/--repo-url flags

Phase 5: Node.js Installation
    ├─> Remove old Node.js if present
    ├─> Install Node.js 20.x LTS from NodeSource
    ├─> Update npm to version 10.x
    └─> Clear npm cache

Phase 6: Build Application
    ├─> Run npm install
    └─> Run npm run build

Phase 7: Zenoh Router & Services Setup
    ├─> Run dcm_zenoh_setup.sh from cloned repo
    ├─> Configure zenoh-router.service
    ├─> Configure rosbridge-websocket.service
    ├─> Configure cmdr-fleet-ui.service
    └─> Create /etc/commander/fleet/fleet.env

Phase 8: Kiosk Display Setup
    ├─> Install OpenBox, LightDM
    ├─> Install Chromium via snap
    ├─> Configure auto-login for developer user
    ├─> Setup Chromium kiosk mode
    └─> Point kiosk to http://localhost:8090
```

### Deployment Script Usage

**Step 1: Clone Deployment Repository**
```bash
sudo apt install gh
gh auth login
sudo mkdir -p /opt/commander/fleet
cd /opt/commander/fleet
sudo git clone https://github.com/Ayanda43/dcm-fleet-deployment.git /opt/commander/fleet/deployment
cd deployment
```

**Step 2: Run Deployment Script**
```bash
sudo ./deploy-dcm-fleet.sh
```

**Note:** The script handles reboots automatically and resumes from the last completed phase using state persistence in `/var/lib/commander/fleet/deployment_state`.

### Repository Selection

You can choose which DCM repository to clone at deployment time:

| Option | Repository | Command |
|--------|------------|---------|
| Default | Ayanda43/tsam-dcm | `sudo ./deploy-dcm-fleet.sh` |
| Corolla | Ayanda43/corrolla-dcm | `sudo ./deploy-dcm-fleet.sh --dcm=corolla` |
| Custom URL | Any git URL | `sudo ./deploy-dcm-fleet.sh --repo-url=<url>` |
| Custom GitHub | owner/repo | `sudo ./deploy-dcm-fleet.sh --repo-url=owner/repo` |

**Notes on cloning behavior:**
- If `--repo-url` begins with `http://` or `https://`, the script uses `git clone`
- If a short form like `owner/repo` is provided, the script uses `gh repo clone`
- All repositories are cloned into `/opt/commander/fleet/dcm-control`

### Manual Steps Required

**Before Running Script:**
1. Complete hardware preparation (see Hardware Preparation section)
2. Authenticate with GitHub using `gh auth login`
3. Ensure proper network connectivity
4. Set hostname if needed: `sudo hostnamectl set-hostname <device-id>`

**During Deployment:**
- Script may prompt for sudo password
- Reboot required after RevPi kernel setup (script handles this)
- Build phase may take 5-10 minutes on ARM64 hardware

**After Deployment:**
- Review `/etc/commander/fleet/fleet.env` configuration
- Test services with `sudo systemctl status zenoh-router rosbridge-websocket cmdr-fleet-ui`
- Verify web UI at `http://localhost:8090`

## Post-Deployment

### Service Management

**Check service status:**
```bash
sudo systemctl status zenoh-router.service
sudo systemctl status rosbridge-websocket.service
sudo systemctl status cmdr-fleet-ui.service
```

**View service logs:**
```bash
sudo journalctl -u zenoh-router.service -f
sudo journalctl -u rosbridge-websocket.service -f
sudo journalctl -u cmdr-fleet-ui.service -f
```

**Restart services:**
```bash
sudo systemctl restart zenoh-router.service
sudo systemctl restart rosbridge-websocket.service
sudo systemctl restart cmdr-fleet-ui.service
```

### Configuration Updates

**Environment Configuration:**
```bash
sudo nano /etc/commander/fleet/fleet.env
# Update configuration values as needed
sudo systemctl restart cmdr-fleet-ui.service
```

**fleet.env contents:**
```bash
# ROS2 Configuration
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ROS_DISTRO=kilted

# DCM Paths
DCM_DIR=/opt/commander/fleet/dcm-control
CONFIG_DIR=/etc/commander/fleet

# Zenoh Configuration
ZENOH_SESSION_CONFIG_URI=/opt/commander/fleet/dcm-control/config/zenoh/session-config.json5

# App Configuration
APP_PORT=8090
```

### Validation Checks

**1. ROS2 Node Check:**
```bash
source /opt/ros/kilted/setup.bash
ros2 node list
# Should show active ROS2 nodes
```

**2. CAN Bus Check:**
```bash
ip link show can0
candump can0
# Should show CAN interface and traffic (if devices connected)
```

**3. Web UI Check:**
```bash
curl http://localhost:8090
# Should return HTML content from the DCM web interface
```

**4. Zenoh Router Check:**
```bash
sudo systemctl status zenoh-router.service
# Should show active (running)
```

**5. Kiosk Display Check:**
```bash
sudo systemctl status lightdm
# Should show active (running)
# Display should show Chromium with DCM interface
```

### Accessing the System

**Web Interface:**
- Control UI: `http://<device-ip>:8090`

**SSH Access:**
```bash
ssh developer@<device-ip>
```

**ROS2 Command Line:**
```bash
ssh developer@<device-ip>
source /opt/ros/kilted/setup.bash
ros2 topic list
ros2 service list
ros2 node list
```

## Troubleshooting

### Common Issues

**Issue: GitHub authentication fails**
```bash
# Solution: Re-authenticate with GitHub CLI
gh auth login
# Follow prompts to authenticate

# Verify authentication
gh auth status

# Alternative: Use SSH keys
ssh-keygen -t ed25519 -C "your_email@example.com"
gh ssh-key add ~/.ssh/id_ed25519.pub
```

**Issue: Build fails with memory errors**
```bash
# Solution 1: Add swap space
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Solution 2: Re-run the build phase
sudo ./deploy-dcm-fleet.sh
```

**Issue: CAN interface not found**
```bash
# Solution: Load CAN kernel modules manually
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

# Configure CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Verify
ip link show can0
```

**Issue: Services fail to start**
```bash
# Check service logs for specific errors
sudo journalctl -u zenoh-router.service -n 50
sudo journalctl -u cmdr-fleet-ui.service -n 50

# Common causes:
# - Missing environment file: Check /etc/commander/fleet/fleet.env exists
# - Dependencies not started: Ensure zenoh-router starts before others
# - Port conflicts: Check if port 8090 is already in use
netstat -tlnp | grep 8090
```

**Issue: Kiosk display not showing**
```bash
# Check LightDM status
sudo systemctl status lightdm

# Check display manager logs
sudo journalctl -u lightdm -n 50

# Verify OpenBox session exists
ls -la /usr/share/xsessions/openbox.desktop

# Check auto-login configuration
cat /etc/lightdm/lightdm.conf | grep autologin

# Restart display manager
sudo systemctl restart lightdm
```

**Issue: Deployment stuck or incomplete**
```bash
# Check current deployment state
cat /var/lib/commander/fleet/deployment_state

# View deployment logs
sudo tail -n 200 /var/log/fleet-deployment.log

# Reset and re-run from beginning
sudo rm /var/lib/commander/fleet/deployment_state
sudo ./deploy-dcm-fleet.sh
```

### Recovery Procedures

**Re-run Deployment:**
```bash
cd /opt/commander/fleet/deployment

# Reset state to re-run from beginning
sudo rm -f /var/lib/commander/fleet/deployment_state

# Run deployment
sudo ./deploy-dcm-fleet.sh
```

**Rebuild Application:**
```bash
cd /opt/commander/fleet/dcm-control
rm -rf node_modules dist build
npm install
npm run build
sudo systemctl restart cmdr-fleet-ui.service
```

**Reset Services:**
```bash
sudo systemctl stop cmdr-fleet-ui.service
sudo systemctl stop rosbridge-websocket.service
sudo systemctl stop zenoh-router.service
sudo systemctl reset-failed
sudo systemctl start zenoh-router.service
sudo systemctl start rosbridge-websocket.service
sudo systemctl start cmdr-fleet-ui.service
```

### Getting Help

For deployment issues:
1. Check deployment logs: `sudo tail -f /var/log/fleet-deployment.log`
2. Check service logs: `sudo journalctl -u <service-name>`
3. Review deployment state: `cat /var/lib/commander/fleet/deployment_state`
4. Open an issue at: https://github.com/Ayanda43/dcm-fleet-deployment/issues

## Deployment Checklist

### Pre-Deployment
- [ ] Hardware setup complete (RevPi Connect 5, CAN, display)
- [ ] Ubuntu 24.04 imaged and booted
- [ ] Network connectivity verified
- [ ] Hostname set appropriately
- [ ] GitHub authentication configured (`gh auth login`)
- [ ] Deployment repository cloned to `/opt/commander/fleet/deployment`

### During Deployment
- [ ] Phase 1: RevPi hardware setup (reboot required)
- [ ] Phase 2: Prerequisites installed (git, curl, gh CLI)
- [ ] Phase 3: ROS2 Kilted installed
- [ ] Phase 4: DCM repository cloned
- [ ] Phase 5: Node.js 20.x installed
- [ ] Phase 6: Application built successfully
- [ ] Phase 7: Zenoh and services configured
- [ ] Phase 8: Kiosk display configured

### Post-Deployment
- [ ] Services running: `sudo systemctl status zenoh-router rosbridge-websocket cmdr-fleet-ui`
- [ ] Web UI accessible: `curl http://localhost:8090`
- [ ] ROS2 environment working: `ros2 node list`
- [ ] CAN bus operational (if applicable): `ip link show can0`
- [ ] Kiosk display showing web interface
- [ ] System reboots cleanly with all services auto-starting

## Version Information

| Component | Version |
|-----------|---------|
| ROS2 Distribution | Kilted |
| Ubuntu Version | 24.04 (Noble) |
| Target Hardware | Revolution Pi Connect 5 (ARM64) |
| Node.js | 20.x LTS |
| npm | 10.x |
| Zenoh RMW | rmw-zenoh-cpp |

---

Last Updated: 2025-01-30
