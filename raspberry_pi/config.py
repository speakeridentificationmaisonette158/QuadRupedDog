"""
Robot Configuration
===================

Central configuration for the quadruped robot controller.
All hardware-specific settings and tunable parameters live here.

Override locally by creating config_local.py (gitignored).
"""

import os

# ============== SERIAL COMMUNICATION ==============

# Serial port for ESP32 connection
# /dev/ttyUSB0  — USB-to-Serial adapter
# /dev/ttyACM0  — ESP32 native USB
# /dev/ttyAMA0  — Raspberry Pi hardware UART (GPIO14/15)
SERIAL_PORT: str = os.environ.get("QUADRUPED_SERIAL_PORT", "/dev/ttyUSB0")
SERIAL_BAUD: int = 115200
SERIAL_TIMEOUT: float = 1.0

# ============== CONTROL LOOP ==============

CONTROL_LOOP_HZ: int = 50  # Main loop frequency

# ============== CAMERA ==============

# Camera source index (0 = default USB camera)
CAMERA_INDEX: int = 0
CAMERA_WIDTH: int = 640
CAMERA_HEIGHT: int = 480
CAMERA_FPS: int = 30

# XIAO ESP32-S3 WiFi stream URL
XIAO_STREAM_URL: str = os.environ.get(
    "QUADRUPED_XIAO_URL", "http://192.168.1.100:81/stream"
)

# ============== WEB SERVER ==============

WEB_HOST: str = "0.0.0.0"
WEB_PORT: int = 5000

# ============== ROBOT GEOMETRY (mm) ==============

# Leg segment lengths
HIP_LENGTH: float = 50.0      # Hip offset from body center to shoulder axis
THIGH_LENGTH: float = 55.0    # Upper leg (shoulder to knee)
SHIN_LENGTH: float = 75.0     # Lower leg (knee to foot)

# Body dimensions (center-to-center of hip joints)
BODY_LENGTH: float = 140.0    # Front-to-rear hip spacing
BODY_WIDTH: float = 80.0      # Left-to-right hip spacing

# Default standing height
STANDING_HEIGHT: float = 80.0  # Ground to shoulder when standing

# ============== SERVO LIMITS ==============

SERVO_MIN_ANGLE: float = -90.0
SERVO_MAX_ANGLE: float = 90.0

# ============== LOCAL OVERRIDES ==============

# Import local overrides if they exist (gitignored)
try:
    from config_local import *  # noqa: F401, F403
except ImportError:
    pass
