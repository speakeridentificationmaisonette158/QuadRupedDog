"""
Vision Module
=============

Camera interface and computer vision for obstacle detection.
"""

from .camera import Camera
from .detection import ObstacleDetector

__all__ = ["Camera", "ObstacleDetector"]
