"""
Serial Bridge - Communication with ESP32 Servo Controller
==========================================================

Handles UART communication between Raspberry Pi and ESP32.
Sends joint angle commands and receives IMU data.
"""

import serial
import time
import threading
from typing import List, Optional, Tuple, Callable
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT


class SerialBridge:
    """
    Manages serial communication with the ESP32 servo controller.
    
    Usage:
        bridge = SerialBridge()
        bridge.connect()
        bridge.set_all_angles([0, 30, -60, 0, 30, -60, 0, 30, -60, 0, 30, -60])
        bridge.disconnect()
    """
    
    def __init__(self, port: str = SERIAL_PORT, baud: int = SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        
        # IMU data
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.imu_lock = threading.Lock()
        
        # Callbacks
        self.imu_callback: Optional[Callable[[float, float, float], None]] = None
        
        # Reader thread
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False
    
    def connect(self) -> bool:
        """
        Establish serial connection to ESP32.
        
        Returns:
            True if connection successful, False otherwise.
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=SERIAL_TIMEOUT,
                write_timeout=SERIAL_TIMEOUT
            )
            
            # ESP32 resets when USB serial opens - wait for boot
            print("[SerialBridge] Waiting for ESP32 to boot...")
            time.sleep(2.5)
            
            # Clear boot messages
            self.serial.reset_input_buffer()
            
            # Send STATUS command
            self.serial.write(b"STATUS\n")
            self.serial.flush()
            
            # Check for response
            start = time.time()
            while time.time() - start < 2.0:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("STATUS:") or line.startswith("OK:") or "READY" in line:
                        self.connected = True
                        print(f"[SerialBridge] Connected to ESP32 on {self.port}")
                        
                        # Start reader thread
                        self._running = True
                        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
                        self._reader_thread.start()
                        
                        return True
                time.sleep(0.05)
            
            print(f"[SerialBridge] ESP32 did not respond")
            self.serial.close()
            return False
            
        except serial.SerialException as e:
            print(f"[SerialBridge] Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close serial connection."""
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        
        if self.serial and self.serial.is_open:
            self.serial.close()
        
        self.connected = False
        print("[SerialBridge] Disconnected")
    
    def _read_loop(self):
        """Background thread to read incoming data from ESP32."""
        while self._running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    self._process_response(line)
                else:
                    time.sleep(0.001)
            except Exception as e:
                if self._running:
                    print(f"[SerialBridge] Read error: {e}")
    
    def _process_response(self, line: str):
        """Process a response line from ESP32."""
        if line.startswith("IMU:"):
            # Parse IMU data: IMU:roll,pitch,yaw
            try:
                parts = line[4:].split(",")
                if len(parts) == 3:
                    with self.imu_lock:
                        self.roll = float(parts[0])
                        self.pitch = float(parts[1])
                        self.yaw = float(parts[2])
                    
                    if self.imu_callback:
                        self.imu_callback(self.roll, self.pitch, self.yaw)
            except ValueError:
                pass
        elif line.startswith("OK:"):
            pass  # Command acknowledged
        elif line.startswith("ERR:"):
            print(f"[SerialBridge] Error from ESP32: {line}")
        elif line.startswith("STATUS:"):
            print(f"[SerialBridge] Status: {line[7:]}")
    
    def _send_command(self, cmd: str) -> bool:
        """
        Send a command to ESP32.
        
        Args:
            cmd: Command string (without newline)
            
        Returns:
            True if sent successfully
        """
        if not self.connected or not self.serial:
            print("[SerialBridge] Not connected")
            return False
        
        try:
            self.serial.write((cmd + "\n").encode('utf-8'))
            self.serial.flush()
            time.sleep(0.01)  # Small delay for ESP32 to process
            return True
        except serial.SerialException as e:
            print(f"[SerialBridge] Send error: {e}")
            return False
    
    def set_all_angles(self, angles: List[float]) -> bool:
        """
        Set all 12 joint angles at once.
        
        Args:
            angles: List of 12 angles in degrees [FL_shoulder, FL_hip, FL_knee, 
                    FR_shoulder, FR_hip, FR_knee, RL_shoulder, RL_hip, RL_knee,
                    RR_shoulder, RR_hip, RR_knee]
                    
        Returns:
            True if command sent successfully
        """
        if len(angles) != 12:
            print(f"[SerialBridge] Expected 12 angles, got {len(angles)}")
            return False
        
        angles_str = ",".join(f"{a:.2f}" for a in angles)
        return self._send_command(f"ANGLES:{angles_str}")
    
    def set_servo(self, index: int, angle: float) -> bool:
        """
        Set a single servo angle.
        
        Args:
            index: Servo index (0-11)
            angle: Angle in degrees
            
        Returns:
            True if command sent successfully
        """
        return self._send_command(f"SERVO:{index},{angle:.2f}")
    
    def set_leg(self, leg_index: int, shoulder: float, hip: float, knee: float) -> bool:
        """
        Set all joints for one leg.
        
        Args:
            leg_index: Leg index (0=FL, 1=FR, 2=RL, 3=RR)
            shoulder: Shoulder angle in degrees (on base plate)
            hip: Hip angle in degrees (on leg)
            knee: Knee angle in degrees (on leg)
            
        Returns:
            True if command sent successfully
        """
        return self._send_command(f"LEG:{leg_index},{shoulder:.2f},{hip:.2f},{knee:.2f}")
    
    def center_all(self) -> bool:
        """
        Center all servos (set all angles to 0).
        
        Returns:
            True if command sent successfully
        """
        return self._send_command("CENTER")
    
    def request_imu(self) -> bool:
        """
        Request IMU data from ESP32.
        
        Returns:
            True if command sent successfully
        """
        return self._send_command("GETIMU")
    
    def get_imu(self) -> Tuple[float, float, float]:
        """
        Get the latest IMU readings.
        
        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        with self.imu_lock:
            return (self.roll, self.pitch, self.yaw)
    
    def set_imu_callback(self, callback: Callable[[float, float, float], None]):
        """
        Set a callback function for IMU updates.
        
        Args:
            callback: Function that takes (roll, pitch, yaw) as arguments
        """
        self.imu_callback = callback
    
    def calibrate_servo(self, index: int, offset: float) -> bool:
        """
        Set the calibration offset for a servo.
        
        Args:
            index: Servo index (0-11)
            offset: Center offset in degrees
            
        Returns:
            True if command sent successfully
        """
        return self._send_command(f"CALIBRATE:{index},{offset:.2f}")
    
    def get_status(self) -> bool:
        """
        Request system status from ESP32.
        
        Returns:
            True if command sent successfully
        """
        return self._send_command("STATUS")


# ============== TEST CODE ==============

if __name__ == "__main__":
    # Test the serial bridge
    print("Testing SerialBridge...")
    
    bridge = SerialBridge()
    
    # Try to connect
    if bridge.connect():
        print("Connected!")
        
        # Get status
        bridge.get_status()
        time.sleep(0.5)
        
        # Center all servos
        print("Centering servos...")
        bridge.center_all()
        time.sleep(1.0)
        
        # Test setting individual servo
        print("Testing servo 0...")
        bridge.set_servo(0, 30)
        time.sleep(0.5)
        bridge.set_servo(0, -30)
        time.sleep(0.5)
        bridge.set_servo(0, 0)
        
        # Test setting a leg
        print("Testing front-left leg...")
        bridge.set_leg(0, 0, 30, -60)
        time.sleep(1.0)
        
        # Request IMU
        print("Requesting IMU...")
        bridge.request_imu()
        time.sleep(0.1)
        roll, pitch, yaw = bridge.get_imu()
        print(f"IMU: roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}")
        
        # Center and disconnect
        bridge.center_all()
        time.sleep(0.5)
        bridge.disconnect()
    else:
        print("Failed to connect!")
