"""
Camera Interface
================

Handles camera input from USB camera, Pi Camera, or XIAO ESP32-S3 stream.
"""

import cv2
import numpy as np
import threading
import time
from typing import Optional, Tuple
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS, XIAO_STREAM_URL


class Camera:
    """
    Camera interface supporting multiple input sources.
    
    Supports:
      - USB cameras (webcam, XIAO via USB)
      - Raspberry Pi Camera Module
      - MJPEG stream from XIAO ESP32-S3 over WiFi
    """
    
    def __init__(self, source=None, width: int = CAMERA_WIDTH, 
                 height: int = CAMERA_HEIGHT, fps: int = CAMERA_FPS):
        """
        Initialize camera.
        
        Args:
            source: Camera source - index (int), stream URL (str), or None for default
            width: Frame width
            height: Frame height
            fps: Target frame rate
        """
        self.source = source if source is not None else CAMERA_INDEX
        self.width = width
        self.height = height
        self.fps = fps
        
        self.cap: Optional[cv2.VideoCapture] = None
        self.frame: Optional[np.ndarray] = None
        self.frame_lock = threading.Lock()
        
        self._running = False
        self._thread: Optional[threading.Thread] = None
    
    def start(self) -> bool:
        """
        Start camera capture.
        
        Returns:
            True if camera started successfully
        """
        try:
            # Open camera source
            if isinstance(self.source, str):
                # Stream URL (MJPEG from XIAO)
                self.cap = cv2.VideoCapture(self.source)
            else:
                # Camera index
                self.cap = cv2.VideoCapture(self.source)
                
                # Set camera properties
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            if not self.cap.isOpened():
                print(f"[Camera] Failed to open source: {self.source}")
                return False
            
            # Start capture thread
            self._running = True
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
            
            print(f"[Camera] Started on source: {self.source}")
            return True
            
        except Exception as e:
            print(f"[Camera] Error starting camera: {e}")
            return False
    
    def stop(self):
        """Stop camera capture."""
        self._running = False
        
        if self._thread:
            self._thread.join(timeout=1.0)
        
        if self.cap:
            self.cap.release()
            self.cap = None
        
        print("[Camera] Stopped")
    
    def _capture_loop(self):
        """Background thread for continuous frame capture."""
        while self._running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            
            if ret:
                with self.frame_lock:
                    self.frame = frame
            else:
                time.sleep(0.01)
    
    def get_frame(self) -> Optional[np.ndarray]:
        """
        Get the latest frame.
        
        Returns:
            Frame as numpy array (BGR), or None if no frame available
        """
        with self.frame_lock:
            if self.frame is not None:
                return self.frame.copy()
        return None
    
    def get_frame_rgb(self) -> Optional[np.ndarray]:
        """
        Get the latest frame in RGB format.
        
        Returns:
            Frame as numpy array (RGB), or None if no frame available
        """
        frame = self.get_frame()
        if frame is not None:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return None
    
    @property
    def is_running(self) -> bool:
        """Check if camera is running."""
        return self._running and self.cap is not None and self.cap.isOpened()


class XIAOCamera(Camera):
    """
    Specialized camera class for XIAO ESP32-S3 Sense over WiFi.
    
    Connects to the MJPEG stream served by the XIAO.
    """
    
    def __init__(self, stream_url: str = XIAO_STREAM_URL, 
                 width: int = 640, height: int = 480):
        """
        Initialize XIAO camera stream.
        
        Args:
            stream_url: URL of the MJPEG stream
            width: Expected frame width
            height: Expected frame height
        """
        super().__init__(source=stream_url, width=width, height=height)
    
    def wait_for_connection(self, timeout: float = 10.0) -> bool:
        """
        Wait for the XIAO stream to become available.
        
        Args:
            timeout: Maximum time to wait (seconds)
            
        Returns:
            True if connection established
        """
        import urllib.request
        import urllib.error
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # Try to connect to the stream
                urllib.request.urlopen(self.source, timeout=1.0)
                return True
            except (urllib.error.URLError, Exception):
                time.sleep(0.5)
        
        return False


# ============== TEST CODE ==============

if __name__ == "__main__":
    print("Testing Camera...")
    
    # Try USB camera first
    cam = Camera(source=0)
    
    if cam.start():
        print("Camera started! Press 'q' to quit.")
        
        # Display frames
        while True:
            frame = cam.get_frame()
            
            if frame is not None:
                cv2.imshow("Camera", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cam.stop()
        cv2.destroyAllWindows()
    else:
        print("Failed to start camera!")
