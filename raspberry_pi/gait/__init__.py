"""
Gait Module
===========

Gait generation and inverse kinematics for quadruped locomotion.
"""

from .kinematics import LegKinematics, QuadrupedKinematics
from .walk import GaitGenerator, GaitType

__all__ = ["LegKinematics", "QuadrupedKinematics", "GaitGenerator", "GaitType"]
