"""
Leg Kinematics
==============

Inverse and forward kinematics for a 3-DOF quadruped leg.

Each leg has three joints:
  - Hip:   rotates the leg laterally (yaw, on the body plate)
  - Thigh: pitches the upper leg segment (shoulder joint)
  - Knee:  pitches the lower leg segment

Coordinate frame (per-leg, right-hand rule):
  +X = forward, +Y = left, +Z = up
  Origin at the hip joint.
"""

import math
import numpy as np
from typing import Tuple, List, Optional
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import (
    HIP_LENGTH, THIGH_LENGTH, SHIN_LENGTH,
    BODY_LENGTH, BODY_WIDTH, STANDING_HEIGHT,
    SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
)


class LegKinematics:
    """
    Inverse kinematics solver for a single 3-DOF leg.

    Given a desired foot position (x, y, z) relative to the hip joint,
    computes the joint angles (hip, thigh, knee) in degrees.
    """

    def __init__(
        self,
        hip_length: float = HIP_LENGTH,
        thigh_length: float = THIGH_LENGTH,
        shin_length: float = SHIN_LENGTH,
    ):
        self.hip_length = hip_length
        self.thigh_length = thigh_length
        self.shin_length = shin_length

    def inverse(self, x: float, y: float, z: float) -> Optional[Tuple[float, float, float]]:
        """
        Compute joint angles for a desired foot position.

        Args:
            x: Forward offset from hip (mm)
            y: Lateral offset from hip (mm, positive = outward)
            z: Vertical offset from hip (mm, negative = downward)

        Returns:
            (hip_deg, thigh_deg, knee_deg) or None if unreachable.
        """
        # Hip angle: rotation in the horizontal plane
        hip_rad = math.atan2(y, x) if (abs(x) > 1e-6 or abs(y) > 1e-6) else 0.0

        # Distance from hip axis to foot projected onto the leg plane
        d_xy = math.sqrt(x * x + y * y)

        # Effective reach in the leg plane (subtract hip offset)
        dx = d_xy - self.hip_length
        dz = z

        # Distance from shoulder to foot in the leg plane
        reach = math.sqrt(dx * dx + dz * dz)

        # Check reachability
        max_reach = self.thigh_length + self.shin_length
        min_reach = abs(self.thigh_length - self.shin_length)

        if reach > max_reach or reach < min_reach:
            return None

        # Knee angle via law of cosines
        cos_knee = (
            self.thigh_length ** 2 + self.shin_length ** 2 - reach ** 2
        ) / (2 * self.thigh_length * self.shin_length)
        cos_knee = max(-1.0, min(1.0, cos_knee))
        knee_rad = math.acos(cos_knee) - math.pi  # Negative = bent

        # Thigh angle
        alpha = math.atan2(-dz, dx)
        cos_beta = (
            self.thigh_length ** 2 + reach ** 2 - self.shin_length ** 2
        ) / (2 * self.thigh_length * reach)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        thigh_rad = alpha - beta

        # Convert to degrees
        hip_deg = math.degrees(hip_rad)
        thigh_deg = math.degrees(thigh_rad)
        knee_deg = math.degrees(knee_rad)

        # Clamp to servo limits
        hip_deg = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, hip_deg))
        thigh_deg = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, thigh_deg))
        knee_deg = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, knee_deg))

        return (hip_deg, thigh_deg, knee_deg)

    def forward(self, hip_deg: float, thigh_deg: float, knee_deg: float) -> Tuple[float, float, float]:
        """
        Compute foot position from joint angles.

        Args:
            hip_deg:   Hip angle in degrees
            thigh_deg: Thigh angle in degrees
            knee_deg:  Knee angle in degrees

        Returns:
            (x, y, z) foot position in mm relative to hip joint.
        """
        hip_rad = math.radians(hip_deg)
        thigh_rad = math.radians(thigh_deg)
        knee_rad = math.radians(knee_deg)

        # Position in the leg plane
        total_angle = thigh_rad + knee_rad + math.pi
        px = self.thigh_length * math.cos(thigh_rad) + self.shin_length * math.cos(total_angle)
        pz = -(self.thigh_length * math.sin(thigh_rad) + self.shin_length * math.sin(total_angle))

        # Add hip offset and rotate by hip angle
        reach = self.hip_length + px
        x = reach * math.cos(hip_rad)
        y = reach * math.sin(hip_rad)
        z = pz

        return (x, y, z)


class QuadrupedKinematics:
    """
    Full-body kinematics for a four-legged robot.

    Manages four LegKinematics instances and maps between body-frame
    foot positions and the 12 joint angles sent to the ESP32.

    Leg order matches ESP32 servo mapping:
      0 = Front-Left,  1 = Front-Right,
      2 = Rear-Left,   3 = Rear-Right
    """

    # Hip joint positions relative to body center (x_offset, y_sign)
    LEG_OFFSETS = [
        (+1, +1),   # Front-Left:  +x (front), +y (left)
        (+1, -1),   # Front-Right: +x (front), -y (right)
        (-1, +1),   # Rear-Left:   -x (rear),  +y (left)
        (-1, -1),   # Rear-Right:  -x (rear),  -y (right)
    ]

    def __init__(self):
        self.legs = [LegKinematics() for _ in range(4)]
        self.body_length = BODY_LENGTH
        self.body_width = BODY_WIDTH

    def body_to_leg_frame(
        self, leg_index: int, x: float, y: float, z: float
    ) -> Tuple[float, float, float]:
        """
        Convert a foot position from body frame to the leg's local frame.

        Args:
            leg_index: 0=FL, 1=FR, 2=RL, 3=RR
            x, y, z:   Foot position in body frame (mm)

        Returns:
            (lx, ly, lz) in the leg's local frame.
        """
        x_sign, y_sign = self.LEG_OFFSETS[leg_index]
        lx = x - x_sign * (self.body_length / 2)
        ly = (y - y_sign * (self.body_width / 2)) * y_sign
        lz = z
        return (lx, ly, lz)

    def get_joint_angles(
        self, foot_positions: List[Tuple[float, float, float]]
    ) -> Optional[List[float]]:
        """
        Compute all 12 joint angles from four foot positions in body frame.

        Args:
            foot_positions: List of 4 (x, y, z) tuples, one per leg.

        Returns:
            List of 12 angles [FL_hip, FL_thigh, FL_knee, FR_..., RL_..., RR_...]
            or None if any leg is unreachable.
        """
        angles: List[float] = []

        for i, (fx, fy, fz) in enumerate(foot_positions):
            lx, ly, lz = self.body_to_leg_frame(i, fx, fy, fz)
            result = self.legs[i].inverse(lx, ly, lz)
            if result is None:
                return None
            angles.extend(result)

        return angles

    def get_standing_positions(
        self, height: float = STANDING_HEIGHT
    ) -> List[Tuple[float, float, float]]:
        """
        Compute foot positions for a neutral standing pose.

        Args:
            height: Standing height in mm (ground to shoulder).

        Returns:
            List of 4 (x, y, z) foot positions in body frame.
        """
        positions = []
        for x_sign, y_sign in self.LEG_OFFSETS:
            fx = x_sign * (self.body_length / 2)
            fy = y_sign * (self.body_width / 2 + HIP_LENGTH)
            fz = -height
            positions.append((fx, fy, fz))
        return positions
