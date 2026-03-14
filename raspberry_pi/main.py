#!/usr/bin/env python3
"""
Quadruped Robot - Main Controller
==================================

Main control loop that coordinates:
  - Gait generation
  - Serial communication with ESP32
  - Camera/vision processing
  - User input handling

Usage:
    python main.py [--no-camera] [--test-mode]
"""

import sys
import time
import argparse
import signal
from typing import Optional

# Local imports
from config import CONTROL_LOOP_HZ, SERIAL_PORT
from communication import SerialBridge
from gait import GaitGenerator, GaitType
from vision import Camera, ObstacleDetector


class QuadrupedController:
    """
    Main controller for the quadruped robot.
    
    Coordinates all subsystems and runs the main control loop.
    """
    
    def __init__(self, use_camera: bool = True, test_mode: bool = False):
        """
        Initialize the controller.
        
        Args:
            use_camera: Whether to enable camera/vision
            test_mode: If True, run without hardware (for testing)
        """
        self.test_mode = test_mode
        self.use_camera = use_camera
        
        # Subsystems
        self.serial: Optional[SerialBridge] = None
        self.gait = GaitGenerator()
        self.camera: Optional[Camera] = None
        self.detector: Optional[ObstacleDetector] = None
        
        # Control state
        self.running = False
        self.loop_rate = CONTROL_LOOP_HZ
        self.loop_period = 1.0 / self.loop_rate
        
        # Movement commands
        self.cmd_forward = 0.0
        self.cmd_strafe = 0.0
        self.cmd_turn = 0.0
        
        # Autonomous mode
        self.autonomous = False
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        print("\n[Controller] Shutdown requested...")
        self.stop()
    
    def start(self) -> bool:
        """
        Initialize and start all subsystems.
        
        Returns:
            True if all systems started successfully
        """
        print("[Controller] Starting quadruped controller...")
        
        # Initialize serial communication
        if not self.test_mode:
            self.serial = SerialBridge(port=SERIAL_PORT)
            if not self.serial.connect():
                print("[Controller] Failed to connect to ESP32!")
                print("[Controller] Running in test mode...")
                self.test_mode = True
                self.serial = None
        
        # Initialize camera
        if self.use_camera:
            self.camera = Camera()
            if self.camera.start():
                self.detector = ObstacleDetector(
                    frame_width=self.camera.width,
                    frame_height=self.camera.height
                )
                print("[Controller] Camera started")
            else:
                print("[Controller] Camera failed to start, continuing without vision")
                self.camera = None
        
        # Set initial pose
        self.gait.set_gait(GaitType.STAND)
        self._send_angles(self.gait.get_standing_angles())
        
        self.running = True
        print("[Controller] Ready!")
        
        return True
    
    def stop(self):
        """Stop all subsystems and shutdown."""
        print("[Controller] Stopping...")
        self.running = False
        
        # Stop movement
        self.gait.set_gait(GaitType.STAND)
        self.gait.set_velocity(0, 0, 0)
        
        # Return to standing pose
        if self.serial:
            angles = self.gait.get_standing_angles()
            if angles:
                self._send_angles(angles)
            time.sleep(0.5)
            self.serial.disconnect()
        
        # Stop camera
        if self.camera:
            self.camera.stop()
        
        print("[Controller] Stopped")
    
    def _send_angles(self, angles: list) -> bool:
        """
        Send joint angles to ESP32.
        
        Args:
            angles: List of 12 joint angles
            
        Returns:
            True if sent successfully
        """
        if self.test_mode or not self.serial:
            return True
        
        return self.serial.set_all_angles(angles)
    
    def set_velocity(self, forward: float = 0.0, strafe: float = 0.0, turn: float = 0.0):
        """
        Set movement velocity commands.
        
        Args:
            forward: Forward speed (-1 to 1)
            strafe: Strafe speed (-1 to 1)
            turn: Turn rate (-1 to 1)
        """
        self.cmd_forward = forward
        self.cmd_strafe = strafe
        self.cmd_turn = turn
        self.gait.set_velocity(forward, strafe, turn)
    
    def set_gait(self, gait_type: GaitType):
        """
        Set the walking gait pattern.
        
        Args:
            gait_type: The gait to use
        """
        self.gait.set_gait(gait_type)
    
    def _process_vision(self):
        """Process camera frame and detect obstacles."""
        if not self.camera or not self.detector:
            return
        
        frame = self.camera.get_frame()
        if frame is None:
            return
        
        # Detect obstacles
        detected, direction, distance = self.detector.detect_ground_obstacles(frame)
        
        if self.autonomous and detected:
            # Simple obstacle avoidance
            if distance < 0.3:
                # Too close - stop and back up
                self.set_velocity(forward=-0.3, turn=-direction * 0.5)
            elif distance < 0.5:
                # Getting close - slow down and turn away
                self.set_velocity(forward=0.3, turn=-direction * 0.5)
    
    def run(self):
        """Main control loop."""
        print("[Controller] Starting main loop...")
        print("[Controller] Press Ctrl+C to stop")
        
        last_time = time.time()
        loop_count = 0
        
        while self.running:
            loop_start = time.time()
            
            # Process vision
            if self.use_camera:
                self._process_vision()
            
            # Update gait and get joint angles
            angles = self.gait.update()
            
            if angles:
                self._send_angles(angles)
            
            # Request IMU data periodically
            if self.serial and loop_count % 5 == 0:
                self.serial.request_imu()
            
            loop_count += 1
            
            # Print status every second
            if time.time() - last_time >= 1.0:
                if self.serial:
                    roll, pitch, yaw = self.serial.get_imu()
                    print(f"[Controller] Gait: {self.gait.gait_type.value}, "
                          f"IMU: R={roll:.1f} P={pitch:.1f} Y={yaw:.1f}")
                else:
                    print(f"[Controller] Gait: {self.gait.gait_type.value} (test mode)")
                last_time = time.time()
            
            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = self.loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def run_interactive(self):
        """
        Run with keyboard control.
        
        Uses simple terminal input for control.
        """
        print("\n=== Quadruped Robot Control ===")
        print("Commands:")
        print("  w/s     - Forward/Backward")
        print("  a/d     - Turn left/right")
        print("  q/e     - Strafe left/right")
        print("  1       - Stand")
        print("  2       - Trot gait")
        print("  3       - Crawl gait")
        print("  space   - Stop")
        print("  x       - Exit")
        print("================================\n")
        
        import threading
        
        # Start control loop in background
        control_thread = threading.Thread(target=self.run, daemon=True)
        control_thread.start()
        
        # Simple keyboard input (blocking)
        try:
            while self.running:
                cmd = input().strip().lower()
                
                if cmd == 'w':
                    self.set_velocity(forward=0.5)
                    self.set_gait(GaitType.TROT)
                elif cmd == 's':
                    self.set_velocity(forward=-0.5)
                    self.set_gait(GaitType.TROT)
                elif cmd == 'a':
                    self.set_velocity(turn=-0.5)
                    self.set_gait(GaitType.TROT)
                elif cmd == 'd':
                    self.set_velocity(turn=0.5)
                    self.set_gait(GaitType.TROT)
                elif cmd == 'q':
                    self.set_velocity(strafe=-0.5)
                    self.set_gait(GaitType.TROT)
                elif cmd == 'e':
                    self.set_velocity(strafe=0.5)
                    self.set_gait(GaitType.TROT)
                elif cmd == ' ' or cmd == '':
                    self.set_velocity(0, 0, 0)
                elif cmd == '1':
                    self.set_gait(GaitType.STAND)
                    self.set_velocity(0, 0, 0)
                elif cmd == '2':
                    self.set_gait(GaitType.TROT)
                elif cmd == '3':
                    self.set_gait(GaitType.CRAWL)
                elif cmd == 'x':
                    break
        except EOFError:
            pass
        
        self.stop()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="Quadruped Robot Controller")
    parser.add_argument("--no-camera", action="store_true",
                       help="Disable camera/vision")
    parser.add_argument("--test-mode", action="store_true",
                       help="Run without hardware for testing")
    parser.add_argument("--interactive", action="store_true",
                       help="Run with keyboard control")
    args = parser.parse_args()
    
    controller = QuadrupedController(
        use_camera=not args.no_camera,
        test_mode=args.test_mode
    )
    
    if controller.start():
        if args.interactive:
            controller.run_interactive()
        else:
            controller.run()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
