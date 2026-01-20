import os
import logging
import atexit 
import math
logging_format = '%(asctime)s: %(message)s'
logging.basicConfig(format=logging_format, level=logging.INFO,
datefmt='%H:%M:%S')
logging.getLogger().setLevel(logging.DEBUG)
from logdecorator import log_on_start, log_on_end, log_on_error
try:
    on_the_robot = os.uname().machine.startswith('arm') or os.uname().machine.startswith('aarch64')
    if not on_the_robot:
        raise ImportError
    from robot_hat import Pin, ADC, PWM, Servo, fileDB
    from robot_hat import Grayscale_Module, Ultrasonic, utils
except ImportError:
    import sys
    sys.path.append(os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..')))
    from sim_robot_hat import Pin, ADC, PWM, Servo, fileDB
    from sim_robot_hat import Grayscale_Module, Ultrasonic, utils
import time




class Picarx(object):
    CONFIG = '/opt/picar-x/picar-x.conf'

    DEFAULT_LINE_REF = [1000, 1000, 1000]
    DEFAULT_CLIFF_REF = [500, 500, 500]

    DIR_MIN = -30
    DIR_MAX = 30
    CAM_PAN_MIN = -90
    CAM_PAN_MAX = 90
    CAM_TILT_MIN = -35
    CAM_TILT_MAX = 65

    PERIOD = 4095
    PRESCALER = 10
    TIMEOUT = 0.02

    # servo_pins: camera_pan_servo, camera_tilt_servo, direction_servo
    # motor_pins: left_swicth, right_swicth, left_pwm, right_pwm
    # grayscale_pins: 3 adc channels
    # ultrasonic_pins: trig, echo2
    # config: path of config file
    def __init__(self, 
                servo_pins:list=['P0', 'P1', 'P2'], 
                motor_pins:list=['D4', 'D5', 'P13', 'P12'],
                grayscale_pins:list=['A0', 'A1', 'A2'],
                ultrasonic_pins:list=['D2','D3'],
                config:str=CONFIG,
                ):

        # reset robot_hat
        utils.reset_mcu()
        time.sleep(0.2)

        # --------- config_flie ---------
        self.config_flie = fileDB(config, 777, os.getlogin())

        # --------- servos init ---------
        self.cam_pan = Servo(servo_pins[0])
        self.cam_tilt = Servo(servo_pins[1])   
        self.dir_servo_pin = Servo(servo_pins[2])
        # get calibration values
        self.dir_cali_val = float(self.config_flie.get("picarx_dir_servo", default_value=0))
        self.cam_pan_cali_val = float(self.config_flie.get("picarx_cam_pan_servo", default_value=0))
        self.cam_tilt_cali_val = float(self.config_flie.get("picarx_cam_tilt_servo", default_value=0))
        # set servos to init angle
        self.dir_servo_pin.angle(self.dir_cali_val)
        self.cam_pan.angle(self.cam_pan_cali_val)
        self.cam_tilt.angle(self.cam_tilt_cali_val)

        # --------- motors init ---------
        self.left_rear_dir_pin = Pin(motor_pins[0])
        self.right_rear_dir_pin = Pin(motor_pins[1])
        self.left_rear_pwm_pin = PWM(motor_pins[2])
        self.right_rear_pwm_pin = PWM(motor_pins[3])
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        # get calibration values
        self.cali_dir_value = self.config_flie.get("picarx_dir_motor", default_value="[1, 1]")
        self.cali_dir_value = [int(i.strip()) for i in self.cali_dir_value.strip().strip("[]").split(",")]
        self.cali_speed_value = [0, 0]
        self.dir_current_angle = 0
        # init pwm
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

        # --------- grayscale module init ---------
        adc0, adc1, adc2 = [ADC(pin) for pin in grayscale_pins]
        self.grayscale = Grayscale_Module(adc0, adc1, adc2, reference=None)
        # get reference
        self.line_reference = self.config_flie.get("line_reference", default_value=str(self.DEFAULT_LINE_REF))
        self.line_reference = [float(i) for i in self.line_reference.strip().strip('[]').split(',')]
        self.cliff_reference = self.config_flie.get("cliff_reference", default_value=str(self.DEFAULT_CLIFF_REF))
        self.cliff_reference = [float(i) for i in self.cliff_reference.strip().strip('[]').split(',')]
        # transfer reference
        self.grayscale.reference(self.line_reference)

        # --------- ultrasonic init ---------
        trig, echo= ultrasonic_pins
        self.ultrasonic = Ultrasonic(Pin(trig), Pin(echo, mode=Pin.IN, pull=Pin.PULL_DOWN))
        atexit.register(self.cleanup)
        
    def constrain(self, x, min_val, max_val):
        return max(min_val, min(max_val, x))
        
        
    def set_motor_speed(self, motor, speed):
        ''' set motor speed
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param speed: speed
        type speed: int      
        '''
        speed = self.constrain(speed, -100, 100)
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # print(f"direction: {direction}, speed: {speed}")
        if speed != 0:
            speed = int(speed /2 ) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
            
    def ackerman_speed(self, speed):
        logging.debug(f"Ackerman speed called with speed: {speed}, dir angle: {self.dir_current_angle}")
        angle = self.dir_current_angle
        angle = self.constrain(angle, self.DIR_MIN, self.DIR_MAX)
        if abs(angle) < 0.1:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, -speed)
            return
        #trackwidth, wheelbase in mm
        wheel_base = 115
        track_width = 90
        angle_rad = math.radians(angle)
        R = wheel_base / math.tan(angle_rad)
        
        #going right when angle > 0
        if angle > 0:
            left_radius = R - track_width / 2
            right_radius = R + track_width / 2
        else:
            left_radius = R + track_width / 2
            right_radius = R - track_width / 2
        #speed = speed * (r/R)
        #Wheel speeds should be proportional to their path radii v = ωr
        left_speed = speed * (left_radius / R)
        right_speed = speed * (right_radius / R)
        logging.debug(f"Ackerman speeds calculated: left_speed={left_speed}, right_speed={right_speed}")
        self.set_motor_speed(1, left_speed)
        self.set_motor_speed(2, -right_speed)
       
    def motor_speed_calibration(self, value):
        logging.debug(f"Motor speed calibration called with value: {value}")
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibrate(self, motor, value):
        ''' set motor direction calibration value
        
        param motor: motor index, 1 means left motor, 2 means right motor
        type motor: int
        param value: speed
        type value: int
        '''      
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = 1
        elif value == -1:
            self.cali_dir_value[motor] = -1
        self.config_flie.set("picarx_dir_motor", self.cali_dir_value)

    def dir_servo_calibrate(self, value):
        logging.debug(f"Direction servo calibration called with value: {value}")
        self.dir_cali_val = value
        self.config_flie.set("picarx_dir_servo", "%s"%value)
        self.dir_servo_pin.angle(value)

    def set_dir_servo_angle(self, value):
        logging.debug(f"Set direction servo angle called with value: {value}")
        self.dir_current_angle = self.constrain(value, self.DIR_MIN, self.DIR_MAX)
        angle_value  = self.dir_current_angle + self.dir_cali_val
        self.dir_servo_pin.angle(angle_value)

    def cam_pan_servo_calibrate(self, value):
        logging.debug(f"Camera pan servo calibration called with value: {value}")
        self.cam_pan_cali_val = value
        self.config_flie.set("picarx_cam_pan_servo", "%s"%value)
        self.cam_pan.angle(value)

    def cam_tilt_servo_calibrate(self, value):
        logging.debug(f"Camera tilt servo calibration called with value: {value}")
        self.cam_tilt_cali_val = value
        self.config_flie.set("picarx_cam_tilt_servo", "%s"%value)
        self.cam_tilt.angle(value)

    def set_cam_pan_angle(self, value):
        logging.debug(f"Set camera pan angle called with value: {value}")
        value = self.constrain(value, self.CAM_PAN_MIN, self.CAM_PAN_MAX)
        self.cam_pan.angle(-1*(value + -1*self.cam_pan_cali_val))

    def set_cam_tilt_angle(self,value):
        logging.debug(f"Set camera tilt angle called with value: {value}")
        value = self.constrain(value, self.CAM_TILT_MIN, self.CAM_TILT_MAX)
        self.cam_tilt.angle(-1*(value + -1*self.cam_tilt_cali_val))

    def set_power(self, speed):
        logging.debug(f"Set power called with speed: {speed}")
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed):
        logging.debug(f"Backward called with speed: {speed}")
        self.ackerman_speed(-speed)

    def forward(self, speed):
        logging.debug(f"Forward called with speed: {speed}")
        self.ackerman_speed(speed)
            
    def straight_forward(self, speed):
        logging.debug(f"Straight forward called with speed: {speed}")
        self.set_dir_servo_angle(0)
        self.ackerman_speed(speed)
        
    def straight_backward(self, speed):
        self.set_dir_servo_angle(0)
        self.ackerman_speed(-speed)
        
    def park_left(self, speed):
        logging.debug(f"Parallel park left called with speed: {speed}")
        # parallel park to the left
        self.set_dir_servo_angle(-30)
        time.sleep(0.1)
        self.ackerman_speed(-speed)
        time.sleep(1.5)
        self.stop()
        
    def park_right(self, speed):
        logging.debug(f"Parallel park right called with speed: {speed}")
        #parallel park to the right
        self.set_dir_servo_angle(30)
        time.sleep(0.1)
        self.ackerman_speed(-speed)
        time.sleep(1.5)
        self.stop()
        
        self.set_dir_servo_angle(0)
 
    def three_point_turn_right(self, speed=30):
        logging.debug(f"Three point turn right called with speed: {speed}")
        self.set_dir_servo_angle(-30)
        #need to sleep in between to allow servo to reach position
        time.sleep(0.1)
        self.ackerman_speed(speed)
        time.sleep(1.5)
        self.stop()
        
        #need to sleep in between to allow servo to reach position
        #set dir to opposite direction
        self.set_dir_servo_angle(30)
        time.sleep(0.1)
        self.ackerman_speed(-speed)
        time.sleep(1.5)
        self.stop()
        
        self.set_dir_servo_angle(0)
    def three_point_turn_left(self, speed=30):
        logging.debug(f"Three point turn left called with speed: {speed}")
        self.set_dir_servo_angle(30)
        #need to sleep in between to allow servo to reach position
        time.sleep(0.1)
        self.ackerman_speed(speed)
        time.sleep(1.5)
        self.stop()
        
        #need to sleep in between to allow servo to reach position
        #set dir to opposite direction
        self.set_dir_servo_angle(-30)
        time.sleep(0.1)
        self.ackerman_speed(-speed)
        time.sleep(1.5)
        self.stop()
        
        self.set_dir_servo_angle(0)
        
    def stop(self):
        logging.debug("Stop called")
        '''
        Execute twice to make sure it stops
        '''
        for _ in range(2):
            self.motor_speed_pins[0].pulse_width_percent(0)
            self.motor_speed_pins[1].pulse_width_percent(0)
            time.sleep(0.002)

    def get_distance(self):
        return self.ultrasonic.read()

    def set_grayscale_reference(self, value):
        if isinstance(value, list) and len(value) == 3:
            self.line_reference = value
            self.grayscale.reference(self.line_reference)
            self.config_flie.set("line_reference", self.line_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def get_grayscale_data(self):
        return list.copy(self.grayscale.read())

    def get_line_status(self,gm_val_list):
        return self.grayscale.read_status(gm_val_list)

    def set_line_reference(self, value):
        self.set_grayscale_reference(value)

    def get_cliff_status(self,gm_val_list):
        for i in range(0,3):
            if gm_val_list[i]<=self.cliff_reference[i]:
                return True
        return False

    def set_cliff_reference(self, value):
        logging.debug(f"Set cliff reference called with value: {value}")
        if isinstance(value, list) and len(value) == 3:
            self.cliff_reference = value
            self.config_flie.set("cliff_reference", self.cliff_reference)
        else:
            raise ValueError("grayscale reference must be a 1*3 list")

    def reset(self):
        logging.debug("Reset called")
        '''
        Reset servos to center position and stop motors
        '''
        self.stop()
        self.set_dir_servo_angle(0)
        self.set_cam_tilt_angle(0)
        self.set_cam_pan_angle(0)
    
    def cleanup(self):
        logging.debug("Cleanup called")
        '''
        Cleanup function called on exit - ensures motors stop and resources are released
        '''
        self.motor_speed_pins[0].pulse_width_percent(0)
        self.motor_speed_pins[1].pulse_width_percent(0)
        self.ultrasonic.close()

    def close(self):
        logging.debug("Close called")
        self.reset()  
        self.ultrasonic.close()

if __name__ == "__main__":
    px = Picarx()
    px.forward(50)
    logger.info("Moving forward")
    time.sleep(2)
    
    px.backward(50)
    logger.info("Moving backward")
    px.straight_forward(50)
    
    logger.info("Moving straight forward")
    time.sleep(2)
    px.straight_backward(50)
    
    
    logger.info("Moving straight backward")
    time.sleep(2)
    
    px.park_left(50)
    logger.info("Parking left")
    px.park_right(50)
    logger.info("Parking right")
    px.three_point_turn_left(50)
    logger.info("Three point turn left")
    px.three_point_turn_right(50)
    logger.info("Three point turn right")
    time.sleep(1)
    px.stop()
