"""
Obstacle Detection
==================

Simple obstacle detection using OpenCV for autonomous navigation.
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Obstacle:
    """Represents a detected obstacle."""
    x: int          # Center X position in image
    y: int          # Center Y position in image
    width: int      # Bounding box width
    height: int     # Bounding box height
    distance: float # Estimated distance (relative, 0-1)
    area: int       # Contour area in pixels


class ObstacleDetector:
    """
    Detects obstacles in camera frames using computer vision.
    
    Uses a combination of:
      - Color-based segmentation
      - Edge detection
      - Contour analysis
    """
    
    def __init__(self, frame_width: int = 640, frame_height: int = 480):
        """
        Initialize obstacle detector.
        
        Args:
            frame_width: Expected frame width
            frame_height: Expected frame height
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        
        # Detection parameters
        self.min_contour_area = 500   # Minimum obstacle size (pixels²)
        self.blur_size = 5            # Gaussian blur kernel size
        self.canny_low = 50           # Canny edge detection low threshold
        self.canny_high = 150         # Canny edge detection high threshold
        
        # Region of interest (bottom half of image = ground plane)
        self.roi_y_start = frame_height // 3
    
    def detect(self, frame: np.ndarray) -> List[Obstacle]:
        """
        Detect obstacles in a frame.
        
        Args:
            frame: BGR image from camera
            
        Returns:
            List of detected obstacles
        """
        if frame is None:
            return []
        
        obstacles = []
        
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (self.blur_size, self.blur_size), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)
        
        # Dilate edges to connect nearby contours
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(edges, kernel, iterations=2)
        
        # Find contours
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Analyze contours
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area < self.min_contour_area:
                continue
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Skip obstacles above ROI (likely background)
            if y + h < self.roi_y_start:
                continue
            
            # Calculate center
            cx = x + w // 2
            cy = y + h // 2
            
            # Estimate distance based on vertical position
            # Lower in image = closer
            distance = 1.0 - (cy / self.frame_height)
            
            obstacles.append(Obstacle(
                x=cx,
                y=cy,
                width=w,
                height=h,
                distance=distance,
                area=area
            ))
        
        # Sort by distance (closest first)
        obstacles.sort(key=lambda o: o.distance)
        
        return obstacles
    
    def detect_ground_obstacles(self, frame: np.ndarray) -> Tuple[bool, float, float]:
        """
        Simplified obstacle detection for navigation.
        
        Args:
            frame: BGR image from camera
            
        Returns:
            Tuple of (obstacle_detected, obstacle_direction, obstacle_distance)
            - obstacle_detected: True if obstacle found
            - obstacle_direction: -1 (left) to 1 (right), 0 = center
            - obstacle_distance: 0 (close) to 1 (far)
        """
        obstacles = self.detect(frame)
        
        if not obstacles:
            return (False, 0.0, 1.0)
        
        # Get the largest/closest obstacle
        main_obstacle = max(obstacles, key=lambda o: o.area)
        
        # Calculate direction (-1 to 1)
        direction = (main_obstacle.x - self.frame_width / 2) / (self.frame_width / 2)
        
        return (True, direction, main_obstacle.distance)
    
    def draw_obstacles(self, frame: np.ndarray, obstacles: List[Obstacle]) -> np.ndarray:
        """
        Draw detected obstacles on frame for visualization.
        
        Args:
            frame: Original BGR frame
            obstacles: List of detected obstacles
            
        Returns:
            Frame with obstacles drawn
        """
        output = frame.copy()
        
        for obs in obstacles:
            # Color based on distance (red = close, green = far)
            color = (
                int(255 * (1 - obs.distance)),  # B
                int(255 * obs.distance),         # G
                int(255 * (1 - obs.distance))   # R
            )
            
            # Draw bounding box
            x1 = obs.x - obs.width // 2
            y1 = obs.y - obs.height // 2
            x2 = obs.x + obs.width // 2
            y2 = obs.y + obs.height // 2
            
            cv2.rectangle(output, (x1, y1), (x2, y2), color, 2)
            
            # Draw center point
            cv2.circle(output, (obs.x, obs.y), 5, color, -1)
            
            # Draw distance label
            label = f"D: {obs.distance:.2f}"
            cv2.putText(output, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Draw ROI line
        cv2.line(output, (0, self.roi_y_start), 
                (self.frame_width, self.roi_y_start), (255, 255, 0), 1)
        
        return output


# ============== TEST CODE ==============

if __name__ == "__main__":
    print("Testing Obstacle Detector...")
    
    # Create synthetic test image
    test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    test_frame[:] = (200, 200, 200)  # Gray background
    
    # Add some "obstacles" (rectangles)
    cv2.rectangle(test_frame, (100, 300), (200, 400), (50, 50, 50), -1)
    cv2.rectangle(test_frame, (400, 350), (550, 450), (30, 30, 30), -1)
    
    # Detect
    detector = ObstacleDetector()
    obstacles = detector.detect(test_frame)
    
    print(f"\nDetected {len(obstacles)} obstacles:")
    for i, obs in enumerate(obstacles):
        print(f"  {i+1}. Position: ({obs.x}, {obs.y}), "
              f"Size: {obs.width}x{obs.height}, Distance: {obs.distance:.2f}")
    
    # Visualize
    output = detector.draw_obstacles(test_frame, obstacles)
    cv2.imshow("Obstacle Detection Test", output)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
