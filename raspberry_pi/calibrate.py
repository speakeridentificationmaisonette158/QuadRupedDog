#!/usr/bin/env python3
"""
Servo Calibration Tool
======================

Interactive tool to calibrate servo positions and find
the correct offset values for each joint.

Usage:
    python calibrate.py
"""

import sys
import time
from communication import SerialBridge
from config import SERIAL_PORT


def print_menu():
    """Print calibration menu."""
    print("\n" + "="*50)
    print("Quadruped Servo Calibration Tool")
    print("="*50)
    print("\nServo Layout:")
    print("  0-2:  Front-Left  (Hip, Thigh, Knee)")
    print("  3-5:  Front-Right (Hip, Thigh, Knee)")
    print("  6-8:  Rear-Left   (Hip, Thigh, Knee)")
    print("  9-11: Rear-Right  (Hip, Thigh, Knee)")
    print("\nCommands:")
    print("  <number>        - Select servo (0-11)")
    print("  +/- or a/d      - Adjust angle by 5°")
    print("  ++/-- or w/s    - Adjust angle by 1°")
    print("  c               - Center current servo")
    print("  C               - Center ALL servos")
    print("  p               - Print current offsets")
    print("  save            - Save offsets (print to console)")
    print("  q               - Quit")
    print("-"*50)


def main():
    """Main calibration loop."""
    print("Connecting to ESP32...")
    
    bridge = SerialBridge(port=SERIAL_PORT)
    
    if not bridge.connect():
        print("Failed to connect! Make sure ESP32 is connected.")
        print("Running in simulation mode...")
        bridge = None
    
    # Current state
    current_servo = 0
    angles = [0.0] * 12
    offsets = [90.0] * 12  # Default center offsets
    
    # Center all servos
    if bridge:
        bridge.center_all()
    
    print_menu()
    
    try:
        while True:
            # Show current state
            servo_names = ["FL_Hip", "FL_Thigh", "FL_Knee",
                          "FR_Hip", "FR_Thigh", "FR_Knee",
                          "RL_Hip", "RL_Thigh", "RL_Knee",
                          "RR_Hip", "RR_Thigh", "RR_Knee"]
            
            print(f"\nServo {current_servo} ({servo_names[current_servo]}): "
                  f"Angle={angles[current_servo]:.1f}°, Offset={offsets[current_servo]:.1f}°")
            
            cmd = input("> ").strip()
            
            if not cmd:
                continue
            
            # Check for number (servo selection)
            if cmd.isdigit():
                num = int(cmd)
                if 0 <= num < 12:
                    current_servo = num
                    print(f"Selected servo {current_servo}")
                else:
                    print("Invalid servo number (0-11)")
                continue
            
            # Adjust angle
            delta = 0
            if cmd in ['+', 'd']:
                delta = 5
            elif cmd in ['-', 'a']:
                delta = -5
            elif cmd in ['++', 'w']:
                delta = 1
            elif cmd in ['--', 's']:
                delta = -1
            
            if delta != 0:
                angles[current_servo] += delta
                angles[current_servo] = max(-90, min(90, angles[current_servo]))
                
                if bridge:
                    bridge.set_servo(current_servo, angles[current_servo])
                
                print(f"Angle: {angles[current_servo]:.1f}°")
                continue
            
            # Center current servo
            if cmd == 'c':
                angles[current_servo] = 0
                if bridge:
                    bridge.set_servo(current_servo, 0)
                print("Centered")
                continue
            
            # Center all servos
            if cmd == 'C':
                for i in range(12):
                    angles[i] = 0
                if bridge:
                    bridge.center_all()
                print("All servos centered")
                continue
            
            # Set offset from current position
            if cmd == 'o':
                # Current angle becomes the offset adjustment
                offsets[current_servo] = 90 + angles[current_servo]
                print(f"Offset set to {offsets[current_servo]:.1f}° for servo {current_servo}")
                continue
            
            # Print all offsets
            if cmd == 'p':
                print("\nCurrent offsets:")
                for i in range(12):
                    print(f"  Servo {i:2d} ({servo_names[i]:10s}): {offsets[i]:.1f}°")
                continue
            
            # Save offsets
            if cmd == 'save':
                print("\n// Copy this to ESP32 servo_controller.ino:")
                print("float SERVO_OFFSETS[NUM_SERVOS] = {")
                for i in range(0, 12, 3):
                    print(f"  {offsets[i]:.1f}, {offsets[i+1]:.1f}, {offsets[i+2]:.1f},"
                          f"  // {servo_names[i][:2]}")
                print("};")
                continue
            
            # Quit
            if cmd == 'q':
                break
            
            print(f"Unknown command: {cmd}")
    
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted")
    
    finally:
        # Clean up
        if bridge:
            bridge.center_all()
            time.sleep(0.3)
            bridge.disconnect()
    
    print("Calibration complete!")


if __name__ == "__main__":
    main()
