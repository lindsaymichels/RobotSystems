'''tests to make sure parallel parking, straight driving, and 3 point turns work'''

import time
import picarx_improved as picarx
import unittest
import warnings

px = picarx.Picarx()

while True:
    manuever = input("Enter maneuver (parallel, straight, three_point) or 'exit' to quit: ").strip().lower()
    if manuever == 'exit':
        break
    elif manuever == 'parallel':
        direction = input("Enter direction (left or right): ").strip().lower()
        if direction == 'left':
            px.park_left(30)
        elif direction == 'right':
            px.park_right(30)
        else:
            print("Invalid direction. Please enter 'left' or 'right'.")
    elif manuever == 'straight':
        direction = input("Enter direction (forward or backward): ").strip().lower()
        if direction == 'forward':
            px.straight_forward(30)
            time.sleep(2)
            px.set_motor_speed(1, 0)
            px.set_motor_speed(2, 0)
        elif direction == 'backward':
            px.straight_backward(30)
            time.sleep(2)
            px.set_motor_speed(1, 0)
            px.set_motor_speed(2, 0)
        else:
            print("Invalid direction. Please enter 'forward' or 'backward'.")
    else:
        if manuever == 'three_point':
            direction = input("Enter direction (left or right): ").strip().lower()
            if direction == 'left':
                px.three_point_turn_left(30)
            elif direction == 'right':
                px.three_point_turn_right(30)
            else:
                print("Invalid direction. Please enter 'left' or 'right'.")
        else:
            print("Invalid maneuver. Please enter 'parallel', 'straight', or 'three_point'.")
    time.sleep(2)
    
    
px.stop()
        
    
