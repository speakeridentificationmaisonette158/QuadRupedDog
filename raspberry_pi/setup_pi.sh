#!/bin/bash
# Raspberry Pi 5 Setup Script for Quadruped Robot
# ================================================
#
# This script installs all required dependencies and configures
# the Raspberry Pi for the quadruped robot project.
#
# Run with: sudo bash setup_pi.sh

set -e

echo "========================================"
echo "Quadruped Robot - Raspberry Pi 5 Setup"
echo "========================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root: sudo bash setup_pi.sh"
    exit 1
fi

# Get the actual user (not root)
REAL_USER=${SUDO_USER:-$USER}
REAL_HOME=$(eval echo ~$REAL_USER)

echo "[1/6] Updating system packages..."
apt update && apt upgrade -y

echo ""
echo "[2/6] Installing system dependencies..."
apt install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-opencv \
    libopencv-dev \
    libatlas-base-dev \
    libjpeg-dev \
    libpng-dev \
    libhdf5-dev \
    git \
    i2c-tools

echo ""
echo "[3/6] Enabling hardware interfaces..."

# Enable I2C
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "dtparam=i2c_arm=on" >> /boot/firmware/config.txt
    echo "  - I2C enabled"
fi

# Enable Serial (UART)
if ! grep -q "^enable_uart=1" /boot/firmware/config.txt 2>/dev/null; then
    echo "enable_uart=1" >> /boot/firmware/config.txt
    echo "  - UART enabled"
fi

# Disable serial console to free up UART for our use
if [ -f /boot/firmware/cmdline.txt ]; then
    sed -i 's/console=serial0,115200 //g' /boot/firmware/cmdline.txt
    echo "  - Serial console disabled"
fi

# Add user to required groups
usermod -a -G dialout,i2c,gpio,video $REAL_USER
echo "  - User added to dialout, i2c, gpio, video groups"

echo ""
echo "[4/6] Creating Python virtual environment..."
VENV_PATH="$REAL_HOME/QuadRupedDog/raspberry_pi/venv"

if [ ! -d "$VENV_PATH" ]; then
    sudo -u $REAL_USER python3 -m venv "$VENV_PATH"
fi

echo ""
echo "[5/6] Installing Python dependencies..."
sudo -u $REAL_USER "$VENV_PATH/bin/pip" install --upgrade pip
sudo -u $REAL_USER "$VENV_PATH/bin/pip" install -r "$REAL_HOME/QuadRupedDog/raspberry_pi/requirements.txt"

echo ""
echo "[6/6] Creating systemd service (optional)..."

cat > /etc/systemd/system/quadruped.service << EOF
[Unit]
Description=Quadruped Robot Controller
After=network.target

[Service]
Type=simple
User=$REAL_USER
WorkingDirectory=$REAL_HOME/QuadRupedDog/raspberry_pi
ExecStart=$VENV_PATH/bin/python main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
echo "  - Service created (not enabled)"
echo "  - To enable auto-start: sudo systemctl enable quadruped"
echo "  - To start manually: sudo systemctl start quadruped"

echo ""
echo "========================================"
echo "Setup Complete!"
echo "========================================"
echo ""
echo "Next steps:"
echo "  1. Reboot the Pi: sudo reboot"
echo "  2. Activate virtual environment:"
echo "     cd ~/QuadRupedDog/raspberry_pi"
echo "     source venv/bin/activate"
echo "  3. Run the controller:"
echo "     python main.py --test-mode"
echo ""
echo "Hardware connections:"
echo "  - ESP32 TX (GPIO17) -> Pi RX (GPIO15)"
echo "  - ESP32 RX (GPIO16) -> Pi TX (GPIO14)"
echo "  - Common GND"
echo ""
