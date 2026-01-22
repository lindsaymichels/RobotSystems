#!usr/bin/env python3
import sys
import os
import time

from robot_hat.adc import ADC
from robot_hat.pin import Pin
from robot_hat.modules import Grayscale_Module
from picarx_improved import Picarx
from interpreter import Interpreter
from sensor import Sensor

class Controller:
    
    def __init__(self,scaling_factor=1.0, ):
        self.scaling_factor = scaling_factor
        self.picarx = Picarx()
        
    def compute_control_signal(self, position):
        '''call the steering-servo method from your car class so that
it turns the car toward the line. It should also return the commanded steering angle'''
        steering_angle = position * self.scaling_factor 
        steering_angle = max(min(steering_angle, 30), -30)  # Clamp dat shiz
        self.picarx.set_dir_servo_angle(steering_angle)
        return steering_angle
    
    

def follow_line(controller, interpreter, sensor, speed):
    '''
    combines the sensor, interpreter, and controller functions in a loop so that
the wheels automatically steer left or right as you move the car right and left over a dark line in
the floor. may need to adjust sensitivity/polarity
    '''
    try:
        controller.picarx.forward(speed)  
        while True:
            sensor_values = sensor.read()
            print(f"Raw sensor values: {sensor_values}")  # <-- Add this
            position, magnitude = interpreter.main_processing(sensor_values)
            control_signal = controller.compute_control_signal(position)
            print(f"Sensor Values: {sensor_values}, Position: {position}, Magnitude: {magnitude}, Control Signal: {control_signal}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Line following stopped by user.")
        controller.picarx.stop()
        
def main():
    grayscale = Grayscale_Module(pin0=ADC('A0'), pin1=ADC('A1'), pin2=ADC('A2'), reference=1000)
    interpreter = Interpreter(sensitivity=0.15, polarity='dark')  
    controller = Controller(scaling_factor=10.0)
    
    follow_line(controller, interpreter, grayscale, speed=30)
    
if __name__ == "__main__":
    main()
