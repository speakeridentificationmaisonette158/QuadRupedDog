"""
Gait Generator
==============

Generates walking patterns for the quadruped robot.

Supports multiple gait types:
  - STAND:  Static standing pose
  - TROT:   Diagonal pairs move together (fast, stable)
  - CRAWL:  One leg at a time (slow, very stable)

Each gait defines phase offsets for the four legs. During a gait cycle,
each leg alternates between stance (foot on ground, pushing backward)
and swing (foot in air, moving forward).
"""

import math
import time
from enum import Enum
from typing import List, Optional, Tuple
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import STANDING_HEIGHT
from gait.kinematics import QuadrupedKinematics


class GaitType(Enum):
    """Available gait patterns."""
    STAND = "stand"
    TROT = "trot"
    CRAWL = "crawl"


class GaitGenerator:
    """
    Produces joint angles for walking gaits.

    The generator maintains an internal phase clock. On each update() call,
    it advances the phase, computes foot trajectories for all four legs,
    runs inverse kinematics, and returns the 12 joint angles.
    """

    # Phase offsets per leg for each gait (fraction of cycle, 0.0–1.0)
    # Leg order: FL, FR, RL, RR
    GAIT_PHASES = {
        GaitType.TROT: [0.0, 0.5, 0.5, 0.0],   # Diagonal pairs
        GaitType.CRAWL: [0.0, 0.5, 0.25, 0.75],  # One leg at a time
    }

    def __init__(self):
        self.kinematics = QuadrupedKinematics()
        self.gait_type = GaitType.STAND

        # Velocity commands (normalized, -1 to 1)
        self.vx: float = 0.0    # Forward/backward
        self.vy: float = 0.0    # Strafe left/right
        self.vyaw: float = 0.0  # Turn rate

        # Gait parameters
        self.cycle_time: float = 0.6   # Seconds per full gait cycle
        self.step_length: float = 40.0  # Max step length (mm)
        self.step_height: float = 30.0  # Foot lift height (mm)
        self.duty_factor: float = 0.6   # Fraction of cycle foot is on ground

        # Internal state
        self._phase: float = 0.0
        self._last_time: float = time.time()

    def set_gait(self, gait_type: GaitType) -> None:
        """Set the gait pattern."""
        self.gait_type = gait_type

    def set_velocity(
        self, vx: float = 0.0, vy: float = 0.0, vyaw: float = 0.0
    ) -> None:
        """
        Set velocity commands.

        Args:
            vx:   Forward speed (-1 to 1)
            vy:   Strafe speed (-1 to 1)
            vyaw: Turn rate (-1 to 1)
        """
        self.vx = max(-1.0, min(1.0, vx))
        self.vy = max(-1.0, min(1.0, vy))
        self.vyaw = max(-1.0, min(1.0, vyaw))

    def reset(self) -> None:
        """Reset phase clock."""
        self._phase = 0.0
        self._last_time = time.time()

    def get_standing_angles(self, height: float = STANDING_HEIGHT) -> Optional[List[float]]:
        """
        Compute joint angles for a static standing pose.

        Args:
            height: Standing height in mm.

        Returns:
            List of 12 joint angles, or None if unreachable.
        """
        positions = self.kinematics.get_standing_positions(height)
        return self.kinematics.get_joint_angles(positions)

    def update(self) -> Optional[List[float]]:
        """
        Advance the gait by one tick and return joint angles.

        Returns:
            List of 12 joint angles, or None if standing / unreachable.
        """
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        if self.gait_type == GaitType.STAND:
            return self.get_standing_angles()

        # Advance phase
        self._phase += dt / self.cycle_time
        self._phase %= 1.0

        # Get phase offsets for current gait
        phase_offsets = self.GAIT_PHASES.get(self.gait_type, [0, 0, 0, 0])

        # Compute foot positions
        standing = self.kinematics.get_standing_positions()
        foot_positions: List[Tuple[float, float, float]] = []

        for i, (sx, sy, sz) in enumerate(standing):
            leg_phase = (self._phase + phase_offsets[i]) % 1.0
            dx, dy, dz = self._foot_trajectory(leg_phase)
            foot_positions.append((sx + dx, sy + dy, sz + dz))

        return self.kinematics.get_joint_angles(foot_positions)

    def _foot_trajectory(self, phase: float) -> Tuple[float, float, float]:
        """
        Compute foot displacement for a given phase in the gait cycle.

        Phase 0.0–duty_factor: stance (foot on ground, slides backward)
        Phase duty_factor–1.0: swing (foot in air, moves forward)

        Args:
            phase: Current leg phase (0.0 to 1.0)

        Returns:
            (dx, dy, dz) displacement from standing position in mm.
        """
        # Scale step length by velocity
        sx = self.vx * self.step_length
        sy = self.vy * self.step_length

        if phase < self.duty_factor:
            # Stance phase: foot moves backward on ground
            t = phase / self.duty_factor  # 0 to 1 within stance
            dx = sx * (0.5 - t)
            dy = sy * (0.5 - t)
            dz = 0.0
        else:
            # Swing phase: foot lifts and moves forward
            t = (phase - self.duty_factor) / (1.0 - self.duty_factor)  # 0 to 1 within swing
            dx = sx * (-0.5 + t)
            dy = sy * (-0.5 + t)
            # Parabolic lift profile
            dz = self.step_height * 4.0 * t * (1.0 - t)

        return (dx, dy, dz)
