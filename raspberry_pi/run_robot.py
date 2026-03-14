#!/usr/bin/env python3
"""
Simple robot controller - no camera, no OpenCV
"""
import sys
import time
import select
import termios
import tty

from communication import SerialBridge
from gait import GaitGenerator, GaitType

def get_key():
    """Get a single keypress without blocking."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

def main():
    print("=== Quadruped Robot Controller ===")
    print()
    
    # Connect to ESP32
    print("Connecting to ESP32...")
    bridge = SerialBridge()
    if not bridge.connect():
        print("ERROR: Could not connect to ESP32!")
        print("Check UART wiring: Pi TX(GPIO14)->ESP32 RX(GPIO16), Pi RX(GPIO15)->ESP32 TX(GPIO17)")
        return
    
    print("Connected!")
    time.sleep(0.5)
    
    # Initialize gait generator
    gait = GaitGenerator()
    
    # Set up terminal for raw input
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    
    print()
    print("Controls:")
    print("  1 = Stand")
    print("  2 = Trot gait")
    print("  w/s = Forward/Back")
    print("  a/d = Turn left/right")
    print("  SPACE = Stop")
    print("  c = Center all servos")
    print("  x = Exit")
    print()
    
    running = True
    walking = False
    
    try:
        # Start in standing pose
        print("Standing up...")
        angles = gait.get_standing_angles(height=80)
        if angles:
            bridge.set_all_angles(angles)
        
        last_update = time.time()
        
        while running:
            key = get_key()
            
            if key:
                if key == 'x':
                    print("\nExiting...")
                    running = False
                elif key == '1':
                    print("Stand")
                    walking = False
                    gait.set_velocity(0, 0, 0)
                    angles = gait.get_standing_angles(height=80)
                    if angles:
                        bridge.set_all_angles(angles)
                elif key == '2':
                    print("Trot gait")
                    gait.set_gait(GaitType.TROT)
                    walking = True
                    gait.reset()
                elif key == 'w':
                    print("Forward")
                    gait.set_velocity(vx=0.5)
                    walking = True
                elif key == 's':
                    print("Backward")
                    gait.set_velocity(vx=-0.5)
                    walking = True
                elif key == 'a':
                    print("Turn left")
                    gait.set_velocity(vyaw=0.3)
                    walking = True
                elif key == 'd':
                    print("Turn right")
                    gait.set_velocity(vyaw=-0.3)
                    walking = True
                elif key == ' ':
                    print("Stop")
                    walking = False
                    gait.set_velocity(0, 0, 0)
                elif key == 'c':
                    print("Centering servos")
                    bridge.center_all()
            
            # Update gait
            if walking:
                angles = gait.update()
                if angles:
                    bridge.set_all_angles(angles)
                    time.sleep(0.05)  # Wait for ESP32 to process
            
            time.sleep(0.05)  # 20Hz update rate
            
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        # Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Center servos before exit
        print("Centering servos...")
        bridge.center_all()
        bridge.disconnect()
        print("Done!")

if __name__ == "__main__":
    main()
