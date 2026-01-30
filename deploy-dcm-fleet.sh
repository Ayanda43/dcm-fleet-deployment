#!/bin/bash
###############################################################################
# DCM Fleet - Complete Deployment Script
#
# Automated deployment for DCM (Distributed Control Manager) fleet systems
# with automatic reboot handling and state persistence.
#
# Installation:
#   Clone this repo to: /opt/commander/fleet/deployment/
#
# Usage:
#   sudo ./deploy-dcm-fleet.sh           # Interactive deployment
#   sudo ./deploy-dcm-fleet.sh --auto    # Automated deployment
#
# Features:
# - Automatic reboot handling with state persistence
# - GitHub authentication (gh CLI + SSH fallback)
# - Complete ROS2 Kilted installation
# - Node.js app build and configuration
# - Systemd service installation
# - Kiosk mode setup
#
# Author: Based on BATTALION Technologies deployment scripts
# DCM Fleet System
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
ROS_DISTRO="kilted"
FLEET_DIR="/opt/commander/fleet"
DCM_DIR="/opt/commander/fleet/dcm-control"
DEPLOYMENT_DIR="/opt/commander/fleet/deployment"
CONFIG_DIR="/etc/commander/fleet"
STATE_FILE="/var/lib/commander/fleet/deployment_state"
LOG_FILE="/var/log/fleet-deployment.log"

# Deployment phases
PHASE_INIT="init"
PHASE_REVPI="revpi_setup"
PHASE_PREREQS="prerequisites"
PHASE_ROS2="ros2_install"
PHASE_CLONE="clone_repo"
PHASE_NODEJS="nodejs_install"
PHASE_BUILD="build_app"
PHASE_ZENOH="setup_zenoh"
PHASE_KIOSK="setup_kiosk"
PHASE_COMPLETE="complete"

###############################################################################
# Utility Functions
###############################################################################

log() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $1" | tee -a "$LOG_FILE" >&2
}

log_error() {
    echo -e "${RED}[$(date +'%Y-%m-%d %H:%M:%S')] ERROR:${NC} $1" | tee -a "$LOG_FILE" >&2
}

log_warn() {
    echo -e "${YELLOW}[$(date +'%Y-%m-%d %H:%M:%S')] WARNING:${NC} $1" | tee -a "$LOG_FILE" >&2
}

log_info() {
    echo -e "${BLUE}[$(date +'%Y-%m-%d %H:%M:%S')] INFO:${NC} $1" | tee -a "$LOG_FILE" >&2
}

log_phase() {
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════[...]"
    echo -e "${CYAN}║${NC} $1" >&2
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════[...]"
    log "$1"
}

check_root() {
    if [[ $EUID -ne 0 ]]; then
        log_error "This script must be run as root (use sudo)"
        exit 1
    fi
}

save_state() {
    local phase=$1
    mkdir -p "$(dirname "$STATE_FILE")"
    echo "$phase" > "$STATE_FILE"
    log_info "Saved deployment state: $phase"
}

get_state() {
    if [ -f "$STATE_FILE" ]; then
        cat "$STATE_FILE"
    else
        echo "$PHASE_INIT"
    fi
}

clear_state() {
    rm -f "$STATE_FILE"
    log_info "Cleared deployment state"
}

schedule_post_reboot() {
    log_info "Scheduling script to run after reboot..."

    # Create systemd service for post-reboot execution
    cat > /etc/systemd/system/fleet-deployment.service <<EOF
[Unit]
Description=DCM Fleet Deployment (Post-Reboot Continuation)
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=$DEPLOYMENT_DIR/deploy-dcm-fleet.sh --continue
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

    systemctl daemon-reload
    systemctl enable fleet-deployment.service
    log "✓ Post-reboot script scheduled"
}

remove_post_reboot_service() {
    if systemctl is-enabled fleet-deployment.service &>/dev/null; then
        systemctl disable fleet-deployment.service
        rm -f /etc/systemd/system/fleet-deployment.service
        systemctl daemon-reload
        log "✓ Removed post-reboot service"
    fi
}

###############################################################################
# Phase 1: RevPi Hardware Setup (requires reboot)
###############################################################################

phase_revpi_setup() {
    log_phase "PHASE 1: Revolution Pi Hardware Setup"

    # Check if running on actual RevPi hardware
    if ! grep -q "Raspberry Pi" /proc/cpuinfo && ! grep -q "BCM" /proc/cpuinfo; then
        log_warn "Not running on Raspberry Pi/RevPi hardware - skipping RevPi setup"
        save_state "$PHASE_PREREQS"
        return 0
    fi

    log_info "Detected Raspberry Pi/RevPi hardware"

    # Get script directory
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    # Run RevPi kernel setup script
    if [ -f "$SCRIPT_DIR/revpi_kernel_setup.sh" ]; then
        log_info "Running RevPi kernel setup..."
        bash "$SCRIPT_DIR/revpi_kernel_setup.sh" --device-id=$(hostname)
    else
        log_error "RevPi kernel setup script not found: $SCRIPT_DIR/revpi_kernel_setup.sh"
        exit 1
    fi

    save_state "$PHASE_PREREQS"

    # Schedule post-reboot continuation
    schedule_post_reboot

    log ""
    log "══════════════════════════════════════════════════════════"
    log "  REBOOTING SYSTEM for RevPi kernel modules"
    log "  Deployment will automatically continue after reboot..."
    log "══════════════════════════════════════════════════════════"
    log ""

    sleep 5
    reboot
}

###############################################################################
# Phase 2: Prerequisites & Authentication
###############################################################################

phase_prerequisites() {
    log_phase "PHASE 2: Prerequisites & Authentication"

    # Prevent needrestart from restarting services during deployment
    export NEEDRESTART_MODE=a
    export NEEDRESTART_SUSPEND=1
    export DEBIAN_FRONTEND=noninteractive

    # Create necessary directories
    mkdir -p "$(dirname "$LOG_FILE")"
    mkdir -p "$(dirname "$STATE_FILE")"
    mkdir -p "$FLEET_DIR"
    mkdir -p "$CONFIG_DIR"

    # Check internet connectivity (with retry for post-boot timing)
    log_info "Checking internet connectivity..."
    RETRY_COUNT=0
    MAX_RETRIES=12  # 60 seconds total (12 * 5 seconds)
    while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
        if ping -c 1 -W 2 8.8.8.8 &> /dev/null; then
            log "✓ Internet connection verified"
            break
        fi
        RETRY_COUNT=$((RETRY_COUNT + 1))
        if [ $RETRY_COUNT -lt $MAX_RETRIES ]; then
            log_info "Waiting for network... (attempt $RETRY_COUNT/$MAX_RETRIES)"
            sleep 5
        else
            log_error "No internet connection after $MAX_RETRIES attempts. Please check network settings."
            exit 1
        fi
    done

    # Update package list
    log_info "Updating package lists..."
    apt update

    # Install basic tools
    log_info "Installing prerequisite packages..."
    apt install -y --allow-downgrades git curl wget gnupg lsb-release software-properties-common build-essential

    # GitHub CLI (for private repos)
    if ! command -v gh &> /dev/null; then
        log_info "Installing GitHub CLI..."
        curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | \
            gpg --batch --yes --dearmor -o /usr/share/keyrings/githubcli-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | \
            tee /etc/apt/sources.list.d/github-cli.list > /dev/null
        apt update
        apt install -y --allow-downgrades gh
        log "✓ GitHub CLI installed"
    else
        log "✓ GitHub CLI already installed"
    fi

    save_state "$PHASE_ROS2"
    log "✓ Phase 2 complete"
}

###############################################################################
# Phase 3: ROS2 Kilted Installation
###############################################################################

phase_ros2_install() {
    log_phase "PHASE 3: ROS2 Kilted Installation"

    # Prevent needrestart from restarting services during deployment
    export NEEDRESTART_MODE=a
    export NEEDRESTART_SUSPEND=1
    export DEBIAN_FRONTEND=noninteractive

    # Add ROS2 repository
    log_info "Adding ROS2 repository..."
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
        gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
        tee /etc/apt/sources.list.d/ros2.list > /dev/null

    apt update

    # Hold problematic packages (from RevPi setup)
    log_info "Holding libraspberrypi-bin to prevent conflicts..."
    apt-mark hold libraspberrypi-bin 2>/dev/null || true

    # Install ROS2 Kilted (ros-base includes rmw-zenoh-cpp)
    log_info "Installing ROS2 Kilted base (this may take 10-15 minutes)..."
    apt update
    apt upgrade -y --allow-downgrades
    apt install -y --allow-downgrades ros-kilted-ros-base

    # Install Zenoh RMW implementation
    log_info "Installing Zenoh DDS middleware..."
    apt install -y --allow-downgrades ros-kilted-rmw-zenoh-cpp

    # Install rosbridge for WebSocket communication
    log_info "Installing rosbridge server..."
    apt install -y --allow-downgrades ros-kilted-rosbridge-server

    # Install development tools
    log_info "Installing ROS2 development tools..."
    apt install -y --allow-downgrades \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-dev-tools

    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        log_info "Initializing rosdep..."
        rosdep init
    fi

    sudo -u developer rosdep update || true

    # Source ROS2 environment
    log_info "Configuring ROS2 environment..."
    if ! grep -q "source /opt/ros/kilted/setup.bash" /home/developer/.bashrc; then
        echo "source /opt/ros/kilted/setup.bash" >> /home/developer/.bashrc
    fi

    save_state "$PHASE_CLONE"
    log "✓ Phase 3 complete"
}

###############################################################################
# Phase 4: Clone Repository
###############################################################################

phase_clone_repo() {
    log_phase "PHASE 4: Cloning DCM Repository"

    # Create fleet directory
    mkdir -p "$FLEET_DIR"
    chown -R developer:developer "$FLEET_DIR"

    # Clone tsam-dcm repository
    log_info "Cloning Ayanda43/tsam-dcm repository..."
    if [ -d "$DCM_DIR" ]; then
        log_info "Repository already exists, pulling latest..."
        sudo -u developer bash -c "
            cd $DCM_DIR
            git pull
        "
    else
        sudo -u developer bash -c "
            cd $FLEET_DIR
            gh repo clone Ayanda43/tsam-dcm dcm-control
        "
    fi

    log "✓ Repository cloned to $DCM_DIR"

    save_state "$PHASE_NODEJS"
    log "✓ Phase 4 complete"
}

###############################################################################
# Phase 5: Node.js Installation
###############################################################################

phase_nodejs_install() {
    log_phase "PHASE 5: Node.js 20.x Installation"

    # Prevent needrestart from restarting services during deployment
    export NEEDRESTART_MODE=a
    export NEEDRESTART_SUSPEND=1
    export DEBIAN_FRONTEND=noninteractive

    # Check Node.js version
    log_info "Checking Node.js version..."

    NEED_INSTALL=false
    if command -v node &> /dev/null; then
        NODE_VERSION=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
        log_info "Current Node.js version: v$NODE_VERSION"

        if [ "$NODE_VERSION" -lt 20 ]; then
            log_warn "Node.js version too old (v$NODE_VERSION), upgrading to v20 LTS..."
            NEED_INSTALL=true
        else
            log "✓ Node.js version is compatible: $(node --version)"
        fi
    else
        log_info "Node.js not installed"
        NEED_INSTALL=true
    fi

    # Install Node.js if needed
    if [ "$NEED_INSTALL" = true ]; then
        # Remove old apt-installed nodejs and npm
        log_info "Removing old apt-installed Node.js and npm..."
        apt remove -y nodejs npm || true
        apt autoremove -y || true

        # Remove old NodeSource repository if it exists
        rm -f /etc/apt/sources.list.d/nodesource.list

        # Install NodeSource repository for Node.js 20.x LTS
        log_info "Installing Node.js 20.x LTS from NodeSource..."
        curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
        apt update
        apt-get install -y --install-recommends nodejs

        # Verify npm is installed
        if ! command -v npm &> /dev/null; then
            log_warn "npm not found after Node.js installation, installing separately..."
            apt-get install -y npm
        fi
        log "✓ Node.js $(node --version) and npm $(npm --version) installed"
    fi

    # Update npm to version compatible with Node.js 20.x
    log_info "Updating npm to version 10.x..."
    npm install -g npm@10
    log "✓ npm updated to $(npm --version)"

    # Clear npm cache
    log_info "Clearing npm cache..."
    npm cache clean --force
    log "✓ npm cache cleared"

    save_state "$PHASE_BUILD"
    log "✓ Phase 5 complete"
}

###############################################################################
# Phase 6: Build Application
###############################################################################

phase_build_app() {
    log_phase "PHASE 6: Building DCM Application"

    # Build the Node.js app
    log_info "Installing npm dependencies..."
    sudo -u developer bash -c "
        cd $DCM_DIR
        npm install
    "

    log_info "Building application..."
    sudo -u developer bash -c "
        cd $DCM_DIR
        npm run build
    "

    log "✓ Application built successfully"

    save_state "$PHASE_ZENOH"
    log "✓ Phase 6 complete"
}

###############################################################################
# Phase 7: Zenoh Setup (runs dcm_zenoh_setup.sh)
###############################################################################

phase_setup_zenoh() {
    log_phase "PHASE 7: Zenoh Router & Services Setup"

    # Run the existing dcm_zenoh_setup.sh script from the cloned repo
    log_info "Running dcm_zenoh_setup.sh..."

    if [ -f "$DCM_DIR/scripts/dcm_zenoh_setup.sh" ]; then
        chmod +x "$DCM_DIR/scripts/dcm_zenoh_setup.sh"
        sudo -u developer bash -c "
            cd $DCM_DIR
            ./scripts/dcm_zenoh_setup.sh
        "
        log "✓ Zenoh setup complete"
    else
        log_error "dcm_zenoh_setup.sh not found at $DCM_DIR/scripts/dcm_zenoh_setup.sh"
        exit 1
    fi

    # Create fleet environment configuration
    log_info "Creating fleet environment configuration..."
    cat > "$CONFIG_DIR/fleet.env" <<EOF
# DCM Fleet Configuration
# Auto-generated by deployment script on $(date)

# ROS2 Configuration
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ROS_DISTRO=kilted

# DCM Paths
DCM_DIR=$DCM_DIR
CONFIG_DIR=$CONFIG_DIR

# Zenoh Configuration
ZENOH_SESSION_CONFIG_URI=$DCM_DIR/config/zenoh/session-config.json5

# App Configuration
APP_PORT=8090
EOF

    chmod 644 "$CONFIG_DIR/fleet.env"
    log "✓ Fleet environment configuration created"

    save_state "$PHASE_KIOSK"
    log "✓ Phase 7 complete"
}

###############################################################################
# Phase 8: Kiosk Display Setup
###############################################################################

phase_setup_kiosk() {
    log_phase "PHASE 8: Kiosk Display Setup"

    # Prevent needrestart from restarting services during deployment
    export NEEDRESTART_MODE=a
    export NEEDRESTART_SUSPEND=1
    export DEBIAN_FRONTEND=noninteractive

    log_info "Installing kiosk environment packages..."
    apt install -y --allow-downgrades \
        openbox \
        xinput \
        x11-xserver-utils \
        unclutter \
        lightdm

    # Install Chromium via snap for better Pi 5 GPU support
    log_info "Installing Chromium browser via snap..."
    if ! snap list chromium &> /dev/null; then
        snap install chromium
    fi

    # Hold Chromium snap to prevent auto-updates
    log_info "Holding Chromium snap to prevent auto-updates..."
    snap refresh --hold chromium
    log "✓ Chromium snap installed and held"

    # Create openbox autostart configuration
    log_info "Creating openbox autostart configuration..."
    mkdir -p /etc/xdg/openbox

    cat > /etc/xdg/openbox/autostart << 'EOF'
# DCM Fleet - OpenBox Autostart Configuration
# Automatically launches DCM Control UI in kiosk mode

# Disable screen savers and power management
xset s off
xset s noblank
xset -dpms

# Allow quitting X server with CTRL-ALT-Backspace
setxkbmap -option terminate:ctrl_alt_bksp

# Fix chromium crash recovery prompts
sed -i 's/"exited_cleanly":false/"exited_cleanly":true/' ~/.config/chromium/'Local State' 2>/dev/null || true
sed -i 's/"exited_cleanly":false/"exited_cleanly":true/; s/"exit_type":"[^"]"/"exit_type":"Normal"/' ~/.config/chromium/Default/Preferences 2>/dev/null || true

# Start Chromium in kiosk mode pointing to DCM Control
chromium --disable-infobars --kiosk --ozone-platform=x11 --use-gl=desktop --disable-gpu-compositing 'http://localhost:8090'
EOF

    # Create user-specific openbox config
    log_info "Creating user-specific openbox configuration..."
    sudo -u developer mkdir -p /home/developer/.config/openbox

    cat > /home/developer/.config/openbox/autostart << 'EOF'
# DCM Fleet - User OpenBox Autostart

# Disable screen savers and power management
xset s off
xset s noblank
xset -dpms

# Allow quitting X server with CTRL-ALT-Backspace
setxkbmap -option terminate:ctrl_alt_bksp

# Fix chromium crash recovery prompts
sed -i 's/"exited_cleanly":false/"exited_cleanly":true/' ~/.config/chromium/'Local State' 2>/dev/null || true
sed -i 's/"exited_cleanly":false/"exited_cleanly":true/; s/"exit_type":"[^"]"/"exit_type":"Normal"/' ~/.config/chromium/Default/Preferences 2>/dev/null || true

# Start Chromium in kiosk mode pointing to DCM Control
chromium --disable-infobars --kiosk --ozone-platform=x11 --use-gl=desktop --disable-gpu-compositing 'http://localhost:8090'
EOF

    chmod +x /home/developer/.config/openbox/autostart
    chown -R developer:developer /home/developer/.config/openbox

    # Create .xsession for openbox
    log_info "Configuring X session..."
    cat > /home/developer/.xsession << 'EOF'
#!/bin/bash
exec openbox-session
EOF
    chmod +x /home/developer/.xsession
    chown developer:developer /home/developer/.xsession

    # Configure LightDM for auto-login
    log_info "Configuring LightDM display manager..."

    # Backup existing config
    if [ -f "/etc/lightdm/lightdm.conf" ]; then
        cp /etc/lightdm/lightdm.conf "/etc/lightdm/lightdm.conf.backup.$(date +%Y%m%d_%H%M%S)"
    fi

    # Configure autologin
    cat >> /etc/lightdm/lightdm.conf << 'EOF'

[Seat:*]
autologin-user=developer
autologin-user-timeout=0
user-session=openbox
EOF

    # Create OpenBox session file
    log_info "Creating OpenBox session file..."
    cat > /usr/share/xsessions/openbox.desktop << 'EOF'
[Desktop Entry]
Name=Openbox Kiosk
Comment=Openbox window manager for kiosk mode
Exec=openbox-session
TryExec=openbox-session
Icon=openbox
Type=Application
EOF

    # Create LightDM drop-in to wait for app service
    log_info "Creating LightDM drop-in override..."
    mkdir -p /etc/systemd/system/lightdm.service.d
    cat > /etc/systemd/system/lightdm.service.d/wait-for-app.conf << 'EOF'
[Unit]
After=cmdr-fleet-ui.service
Wants=cmdr-fleet-ui.service
EOF
    chmod 644 /etc/systemd/system/lightdm.service.d/wait-for-app.conf
    systemctl daemon-reload

    # Enable LightDM
    systemctl enable lightdm || true

    log "✓ Kiosk display configured for http://localhost:8090"

    save_state "$PHASE_COMPLETE"
    log "✓ Phase 8 complete"
}

###############################################################################
# Phase 9: Deployment Complete
###############################################################################

phase_complete() {
    log_phase "DEPLOYMENT COMPLETE"

    # Remove post-reboot service if it exists
    remove_post_reboot_service

    # Clear state file
    clear_state

    echo ""
    echo -e "${GREEN}═══════════════════════════════════════════════════════════��[...]"
    echo -e "${GREEN}  DCM Fleet System - Deployment Complete${NC}"
    echo -e "${GREEN}═══════════════════════════════════════════════════════════��[...]"
    echo ""
    echo -e "  ${CYAN}Installation Details:${NC}"
    echo -e "    DCM Directory: $DCM_DIR"
    echo -e "    Config: $CONFIG_DIR/fleet.env"
    echo -e "    Logs: $LOG_FILE"
    echo ""
    echo -e "  ${CYAN}Systemd Services:${NC}"
    echo -e "    zenoh-router.service:       $(systemctl is-enabled zenoh-router.service 2>/dev/null || echo 'disabled')"
    echo -e "    rosbridge-websocket.service: $(systemctl is-enabled rosbridge-websocket.service 2>/dev/null || echo 'disabled')"
    echo -e "    cmdr-fleet-ui.service:      $(systemctl is-enabled cmdr-fleet-ui.service 2>/dev/null || echo 'disabled')"
    echo ""
    echo -e "  ${CYAN}Verification Commands:${NC}"
    echo -e "    systemctl status zenoh-router rosbridge-websocket cmdr-fleet-ui"
    echo -e "    curl http://localhost:8090"
    echo ""
    echo -e "  ${CYAN}Next Steps:${NC}"
    echo -e "    1. Reboot to test auto-start: sudo reboot"
    echo -e "    2. View logs: journalctl -u cmdr-fleet-ui -f"
    echo -e "    3. Check kiosk: systemctl status lightdm"
    echo ""
    echo -e "${GREEN}═══════════════════════════════════════════════════════════��[...]"
    echo ""
}

###############################################################################
# Main Execution Flow
###############################################################################

main() {
    check_root

    # Parse arguments
    AUTO_MODE=false
    CONTINUE_MODE=false

    for arg in "$@"; do
        case $arg in
            --auto) AUTO_MODE=true ;;
            --continue) CONTINUE_MODE=true ;;
        esac
    done

    # Show banner
    echo ""
    echo -e "${CYAN}╔═══════════════════════════════════════════════════════════[...]"
    echo -e "${CYAN}║${NC}  DCM Fleet - Complete Deployment System                    ${CYAN}║${NC}"
    echo -e "${CYAN}║${NC}  Distributed Control Manager                               ${CYAN}║${NC}"
    echo -e "${CYAN}╚═══════════════════════════════════════════════════════════[...]"
    echo ""

    # Get current phase
    CURRENT_PHASE=$(get_state)
    log_info "Current deployment phase: $CURRENT_PHASE"

    # Execute phases in sequence
    case $CURRENT_PHASE in
        $PHASE_INIT|$PHASE_REVPI)
            phase_revpi_setup
            ;;
        $PHASE_PREREQS)
            phase_prerequisites
            phase_ros2_install
            phase_clone_repo
            phase_nodejs_install
            phase_build_app
            phase_setup_zenoh
            phase_setup_kiosk
            phase_complete
            ;;
        $PHASE_ROS2)
            phase_ros2_install
            phase_clone_repo
            phase_nodejs_install
            phase_build_app
            phase_setup_zenoh
            phase_setup_kiosk
            phase_complete
            ;;
        $PHASE_CLONE)
            phase_clone_repo
            phase_nodejs_install
            phase_build_app
            phase_setup_zenoh
            phase_setup_kiosk
            phase_complete
            ;;
        $PHASE_NODEJS)
            phase_nodejs_install
            phase_build_app
            phase_setup_zenoh
            phase_setup_kiosk
            phase_complete
            ;;
        $PHASE_BUILD)
            phase_build_app
            phase_setup_zenoh
            phase_setup_kiosk
            phase_complete
            ;;
        $PHASE_ZENOH)
            phase_setup_zenoh
            phase_setup_kiosk
            phase_complete
            ;;
        $PHASE_KIOSK)
            phase_setup_kiosk
            phase_complete
            ;;
        $PHASE_COMPLETE)
            log_info "Deployment already complete"
            phase_complete
            ;;
    esac
}

# Run main function
main "$@"