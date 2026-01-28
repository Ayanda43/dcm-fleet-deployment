#!/bin/bash
###############################################################################
# Revolution Pi - Kernel & Hardware Setup for Ubuntu 24.04
#
# Configures Revolution Pi hardware, kernel modules, and device drivers.
# This script handles ONLY RevPi hardware setup - ROS2 installation is
# handled separately by the deployment script.
#
# Based on BATTALION Technologies RevPi setup
# Author: Nishalan Govender <nishalan.govender@gmail.com>
###############################################################################

set -e  # Exit on error

# Arguments
DEVICE_ID=""

for ARG in "$@"; do
    case $ARG in
        --device-id=*)
            DEVICE_ID="${ARG#*=}" ;;
        --help|-h)
            echo "Revolution Pi Kernel Setup"
            echo "Usage: ./revpi_kernel_setup.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --device-id=IDENTIFIER    Set device hostname (e.g., tnk0012)"
            echo ""
            exit 0 ;;
    esac
done

# Formatting
line() {
    local msg="$1"
    local width="${COLUMNS:-$(tput cols)}"
    printf "\n"
    printf '%*s\n' "$width" '' | tr ' ' '='
    if [[ -n "$msg" ]]; then
        local padding=$(( (width - ${#msg}) / 2 ))
        printf "%*s%s\n" "$padding" '' "$msg"
    fi
    printf '%*s\n' "$width" '' | tr ' ' '='
    printf "\n"
}

# Set device hostname if provided
if [[ -n "$DEVICE_ID" ]]; then
    line "Setting hostname to $DEVICE_ID"
    sudo hostnamectl set-hostname "$DEVICE_ID"
    echo "127.0.1.1 $DEVICE_ID" | sudo tee -a /etc/hosts
fi

# Update and Upgrade System
line "Updating and upgrading system..."
sudo apt update && sudo apt upgrade -y

# Add RevPi and Raspberry Pi Repositories
line "Adding RevPi and Raspberry Pi repositories..."
cat <<EOF | sudo tee /etc/apt/sources.list.d/_revpi.sources
Types: deb
URIs: http://packages.revolutionpi.de/
Suites: bookworm
Components: main contrib
Signed-By: /usr/share/keyrings/_revpi-keyring.gpg

Types: deb
URIs: http://packages.revolutionpi.de/
Suites: bookworm-backports
Components: main contrib
Signed-By: /usr/share/keyrings/_revpi-keyring.gpg
EOF

cat <<EOF | sudo tee /etc/apt/sources.list.d/raspi.sources
Types: deb
URIs: http://archive.raspberrypi.org/debian/
Suites: bookworm
Components: main
Signed-By: /usr/share/keyrings/raspberrypi-archive-keyring.gpg

Types: deb-src
URIs: http://archive.raspberrypi.org/debian/
Suites: bookworm
Components: main
Signed-By: /usr/share/keyrings/raspberrypi-archive-keyring.gpg
EOF

# Download Keyrings
line "Downloading RevPi and Raspberry Pi keyrings..."
sudo apt install curl -y
sudo curl -o /usr/share/keyrings/_revpi-keyring.gpg "https://gitlab.com/revolutionpi/debos-build/-/raw/master/overlays/repositories/usr/share/keyrings/_revpi-keyring.gpg"
wget -qO- http://archive.raspberrypi.org/debian/raspberrypi.gpg.key | sudo gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg
# Update and Upgrade Again
line "Updating system with new repositories..."
sudo apt update && sudo apt upgrade -y

# Install and Remove Old RevPi Repo
line "Installing revpi-repo and removing old repository files..."
sudo apt install -y revpi-repo
sudo rm -rf /etc/apt/sources.list.d/_revpi.sources /usr/share/keyrings/_revpi-keyring.gpg

# Re-add RevPi sources (revpi-repo doesn't create them on Ubuntu)
cat <<EOF | sudo tee /etc/apt/sources.list.d/revpi.sources
Types: deb
URIs: http://packages.revolutionpi.de/
Suites: bookworm
Components: main contrib
Signed-By: /usr/share/keyrings/revpi.gpg

Types: deb
URIs: http://packages.revolutionpi.de/
Suites: bookworm-backports
Components: main contrib
Signed-By: /usr/share/keyrings/revpi.gpg
EOF

sudo apt update

# Bootstrap System Apt
line "Bootstrapping system packages..."
sudo apt install -y raspi-firmware linux-image-revpi-v8 revpi-base-files revpi-firmware revpi-tools

# Install Minimum Required Packages
line "Installing minimum required packages..."
sudo apt install -y busybox network-manager revpi-nm-config firmware-brcm80211 firmware-nxp

# Setup Default User
line "Configuring developer user..."
sudo usermod -aG adm,audio,dialout,input,netdev,plugdev,render,sudo,users,video developer
echo "developer ALL=(ALL) NOPASSWD: ALL" | sudo tee /etc/sudoers.d/developer

# Enable SSH
line "Enabling SSH..."
sudo systemctl enable ssh

# Boot Configuration
line "Setting up boot configuration..."
sudo cp /boot/firmware/overlays/revpi-dt-blob.dtbo /boot/firmware/dt-blob.bin

# Install Basic Packages
line "Installing basic packages..."
sudo apt install -y picontrol pitest pictory-apache revpi-modbus-client revpi-modbus-server cockpit-revpi cockpit-revpi-redirect-apache

# Add User to picontrol
line "Adding developer user to picontrol group..."
sudo adduser developer picontrol

# Install Lite Packages
line "Installing lite packages..."
sudo apt install -y can-utils python3-can libsocketcan-dev python3-revpimodio2 revpipyload bluez-firmware python3-libgpiod python3-schema python3-revpi-device-info revpi-sos-report rfkill opcua-revpi-server mqtt-revpi-client raspberrypi-sys-mods dphys-swapfile apt-listchanges

# Mask Services
line "Masking unnecessary services..."
sudo systemctl mask raspi-config.service cpufrequtils.service

# Disable Swapfile
line "Disabling swapfile..."
sudo systemctl disable dphys-swapfile

# Add Priorities for Ubuntu, RevPi and Raspi
line "Setting APT priorities..."
cat <<EOF | sudo tee /etc/apt/preferences.d/ubuntu-priority
Package: *
Pin: release a=noble
Pin-Priority: 1001
EOF

cat <<EOF | sudo tee /etc/apt/preferences.d/revpi-priority
Package: *
Pin: origin "packages.revolutionpi.de"
Pin-Priority: 900
EOF

cat <<EOF | sudo tee /etc/apt/preferences.d/raspi-priority
Package: *
Pin: release o=Raspberry Pi Foundation
Pin-Priority: -10
EOF

# Setup CAN
line "Configuring CAN settings..."
cat <<EOF | sudo tee /etc/systemd/network/10-can.network
[Match]
Name=can*

[CAN]
BitRate=500K
EOF

cat <<EOF | sudo tee /etc/systemd/network/80-can-txqueue.link
[Match]
OriginalName=can*

[Link]
TransmitQueueLength=1000
EOF

# Update config.txt
line "Updating kernel config.txt..."
cat <<EOF | sudo tee /boot/firmware/config.txt
# For more options and information see
# http://rptl.io/configtxt
# Some settings may impact device functionality. See link above for details

# Enable audio (loads snd_bcm2835)
dtparam=audio=on

# Automatically load initramfs files, if found
auto_initramfs=1

# Enable DRM VC4 V3D driver
dtoverlay=vc4-kms-v3d
max_framebuffers=1

# Don't have the firmware create an initial video= setting in cmdline.txt.
# Use the kernel's default instead.
disable_fw_kms_setup=1

# Run in 64-bit mode
arm_64bit=1

# Disable compensation for displays with overscan
disable_overscan=1

# Run as fast as firmware / board allows
arm_boost=1

[all]
dtoverlay=dwc2,dr_mode=host
dtparam=ant2
EOF

# Update cmdline.txt
line "Updating kernel cmdline.txt..."
cat <<EOF | sudo tee /boot/firmware/cmdline.txt
dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p2 rootfstype=ext4 fsck.repair=yes rootwait nosplash plymouth.ignore-serial-consoles
EOF

# Update Kernel to Run revpi
line "Setting revpi as kernel..."
cat <<EOF | sudo tee -a /etc/flash-kernel/db
Machine: Raspberry Pi Compute Module 5 Rev 1.0
Kernel-Flavors: rpi-v8
EOF

# Flash Kernel
sudo flash-kernel

# RevPi Completion Message
line "RevPi kernel setup completed successfully!"
echo ""
echo "✓ RevPi kernel modules and firmware installed"
echo "✓ piControl configured"
echo "✓ CAN bus configured (500K bitrate)"
echo "✓ Network Manager configured"
echo "✓ Developer user configured with proper groups"
echo "✓ Boot configuration updated"
echo ""
echo "System will reboot automatically in 10 seconds..."
echo "Deployment will continue after reboot."
sleep 10

# Reboot
line "Rebooting for changes to take effect..."
sudo reboot
