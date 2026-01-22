#!usr/bin/env python3
from robot_hat.adc import ADC
from robot_hat.pin import Pin
from robot_hat.modules import Grayscale_Module

class Interpreter():
    
    def __init__(self, sensitivity = 0.5, polarity = 'light'):
        
        self.sensitivity = sensitivity
        #polarity = 1 when line darker than background, -1 when background darker than line
        self.polarity = polarity
        
        
    def main_processing(self, sensor_values):
        '''
        identifies if there is a sharp change between two adjacent sensor
        values (indicative of an edge), and then using the edge location and sign to determine both
        whether the system is to the left or right of being centered, and whether it is very off-center
        or only slightly off-center. '''

        edge_index = -1
        edge_magnitude = 0
        
        threshold = self.sensitivity * max(sensor_values) 
        #iterate through sensor values to find edges   
        for i in range(len(sensor_values) - 1):
            difference = sensor_values[i+1] - sensor_values[i]
            if self.polarity == 'light':
                difference = -difference
                
            if abs(difference) > abs(threshold):
                if abs(difference) > abs(edge_magnitude):
                    edge_magnitude = difference
                    edge_index = i
        if edge_index == -1:
            return 0.0, 0  # no edge detected, assume centered
        position = edge_index + (0.5 if edge_magnitude > 0 else -0.5)
        magnitude = abs(edge_magnitude)
        
        return position, magnitude
    
    def output(self, position, magnitude):
        '''
        This method should take the output of main_processing and convert it into location of robot relative to line on [-1, 1] 
        where 1 is far left, 0 is centered, and -1 is far right.
        ''' 
        num_sensors = 3  # assuming 3 sensors for left, middle, right
        normalized_position = (position - (num_sensors - 1) / 2) / ((num_sensors - 1) / 2)
        return normalized_position, magnitude   
    
        
       