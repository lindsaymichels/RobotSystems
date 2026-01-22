#!usr/bin/env python3
from robot_hat.adc import ADC
from robot_hat.pin import Pin
from robot_hat.modules import Grayscale_Module



class Sensor():
    """
    Sensor sets up ADC structures
    """
    def __init__(self, chn, address=None, *args, **kwargs):
        """
        Initialize a sensor

        :param chn: channel number (0-7/A0-A7)
        :type chn: int/str
        """
        self.left_adc = ADC("A0")
        self.middle_adc = ADC("A1")
        self.right_adc = ADC("A2")
        self.grayscale = Grayscale_Module(self.left_adc,
                                         self.middle_adc,
                                         self.right_adc)
        self.sensor_pins = [self.left_adc, self.middle_adc, self.right_adc]
        
    def read(self):
        ''' poll the three ADC structures into list that it returns'''
        if isinstance(self.sensor_pins, Grayscale_Module):
            return self.sensor_pins.read()
        else:
            return [pin.read() for pin in self.sensor_pins]




