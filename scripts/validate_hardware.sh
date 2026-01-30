#!/bin/bash
#
# Hardware Validation Script for DCM Fleet Deployment
# Validates RevPi hardware, CAN interfaces, ROS2, and other prerequisites
#

set -e

PASS=0
FAIL=0
WARN=0

print_result() {
    local status=$1
    local message=$2
    case $status in
        pass)
            echo -e "  [\033[0;32mPASS\033[0m] $message"
            ((PASS++))
            ;;
        fail)
            echo -e "  [\033[0;31mFAIL\033[0m] $message"
            ((FAIL++))
            ;;
        warn)
            echo -e "  [\033[0;33mWARN\033[0m] $message"
            ((WARN++))
            ;;
    esac
}

echo "=============================================="
echo "  DCM Fleet Hardware Validation"
echo "=============================================="
echo ""

# System Information
echo "=== System Information ==="
echo "  Hostname: $(hostname)"
echo "  Kernel: $(uname -r)"
echo "  Architecture: $(uname -m)"
echo "  OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo ""

# Check Architecture
echo "=== Architecture Check ==="
if [[ "$(uname -m)" == "aarch64" ]]; then
    print_result pass "ARM64 architecture detected"
else
    print_result warn "Expected ARM64, found: $(uname -m)"
fi
echo ""

# Check Ubuntu Version
echo "=== Ubuntu Version Check ==="
if grep -q "24.04" /etc/os-release; then
    print_result pass "Ubuntu 24.04 detected"
else
    version=$(cat /etc/os-release | grep VERSION_ID | cut -d'"' -f2)
    print_result warn "Expected Ubuntu 24.04, found: $version"
fi
echo ""

# Check RevPi Kernel
echo "=== RevPi Kernel Check ==="
if uname -r | grep -q "revpi"; then
    print_result pass "RevPi kernel detected: $(uname -r)"
else
    print_result warn "RevPi kernel not detected (may still work)"
fi
echo ""

# Check piTest (RevPi tool)
echo "=== RevPi Tools Check ==="
if command -v piTest &> /dev/null; then
    print_result pass "piTest command available"

    # Try to detect devices
    if piTest -d &> /dev/null; then
        device_count=$(piTest -d 2>/dev/null | grep -c "module type" || echo "0")
        if [[ "$device_count" -gt 0 ]]; then
            print_result pass "RevPi devices detected: $device_count"
        else
            print_result warn "No RevPi devices detected"
        fi
    fi
else
    print_result warn "piTest not available (RevPi tools not installed)"
fi
echo ""

# Check CAN Interfaces
echo "=== CAN Interface Check ==="
if ip link show can0 &> /dev/null; then
    print_result pass "can0 interface exists"

    # Check if CAN is UP
    if ip link show can0 | grep -q "state UP"; then
        print_result pass "can0 is UP"
    else
        print_result warn "can0 exists but is DOWN"
    fi
else
    print_result fail "can0 interface not found"
fi

if ip link show can1 &> /dev/null; then
    print_result pass "can1 interface exists"
else
    print_result warn "can1 interface not found (optional)"
fi
echo ""

# Check CAN Kernel Modules
echo "=== CAN Kernel Modules Check ==="
for module in can can_raw can_dev; do
    if lsmod | grep -q "^$module"; then
        print_result pass "Kernel module loaded: $module"
    else
        print_result warn "Kernel module not loaded: $module"
    fi
done
echo ""

# Check Network Connectivity
echo "=== Network Connectivity Check ==="
if ping -c 1 -W 3 8.8.8.8 &> /dev/null; then
    print_result pass "Internet connectivity (ping 8.8.8.8)"
else
    print_result fail "No internet connectivity"
fi

if ping -c 1 -W 3 github.com &> /dev/null; then
    print_result pass "GitHub accessible"
else
    print_result fail "Cannot reach github.com"
fi

if ping -c 1 -W 3 packages.ros.org &> /dev/null; then
    print_result pass "ROS2 packages accessible"
else
    print_result warn "Cannot reach packages.ros.org"
fi
echo ""

# Check ROS2 Installation
echo "=== ROS2 Check ==="
if [[ -f /opt/ros/kilted/setup.bash ]]; then
    print_result pass "ROS2 Kilted installation found"

    # Check ROS2 works
    if source /opt/ros/kilted/setup.bash && ros2 --help &> /dev/null; then
        print_result pass "ROS2 CLI functional"
    else
        print_result warn "ROS2 installed but CLI not working"
    fi
else
    print_result warn "ROS2 Kilted not installed (will be installed during deployment)"
fi
echo ""

# Check Node.js
echo "=== Node.js Check ==="
if command -v node &> /dev/null; then
    node_version=$(node --version)
    if [[ "$node_version" == v20* ]]; then
        print_result pass "Node.js 20.x installed: $node_version"
    else
        print_result warn "Node.js installed but not v20.x: $node_version"
    fi
else
    print_result warn "Node.js not installed (will be installed during deployment)"
fi
echo ""

# Check Memory
echo "=== Memory Check ==="
total_mem=$(free -m | awk '/^Mem:/{print $2}')
if [[ $total_mem -ge 4000 ]]; then
    print_result pass "Sufficient RAM: ${total_mem}MB"
else
    print_result warn "Low RAM: ${total_mem}MB (4GB recommended)"
fi
echo ""

# Check Disk Space
echo "=== Disk Space Check ==="
available_space=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')
if [[ $available_space -ge 10 ]]; then
    print_result pass "Sufficient disk space: ${available_space}GB available"
else
    print_result fail "Low disk space: ${available_space}GB (10GB+ recommended)"
fi
echo ""

# Check Display
echo "=== Display Check ==="
if [[ -n "$DISPLAY" ]]; then
    print_result pass "DISPLAY variable set: $DISPLAY"
else
    print_result warn "DISPLAY not set (headless mode)"
fi

if command -v lightdm &> /dev/null; then
    print_result pass "LightDM installed"
else
    print_result warn "LightDM not installed (will be installed for kiosk mode)"
fi
echo ""

# Summary
echo "=============================================="
echo "  Validation Summary"
echo "=============================================="
echo -e "  \033[0;32mPassed:\033[0m $PASS"
echo -e "  \033[0;33mWarnings:\033[0m $WARN"
echo -e "  \033[0;31mFailed:\033[0m $FAIL"
echo "=============================================="
echo ""

if [[ $FAIL -gt 0 ]]; then
    echo "Some checks failed. Please review the issues above before deployment."
    exit 1
elif [[ $WARN -gt 0 ]]; then
    echo "Some warnings detected. Deployment may still proceed."
    exit 0
else
    echo "All checks passed! System is ready for deployment."
    exit 0
fi
