#!/usr/bin/python3
import sys
import math
import time
import threading
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[1]
if str(BASE_DIR) not in sys.path:
    sys.path.append(str(BASE_DIR))

import cv2
import numpy as np

try:
    import Camera
except ModuleNotFoundError:
    import camera as Camera

from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

RANGE_RGB = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


def get_area_max_contour(contours):
    contour_area_max = 0
    area_max_contour = None
    for contour in contours:
        contour_area = math.fabs(cv2.contourArea(contour))
        if contour_area > contour_area_max:
            contour_area_max = contour_area
            if contour_area > 300:
                area_max_contour = contour
    return area_max_contour, contour_area_max


class Perception:
    def __init__(self):
        self.size = (640, 480)
        self.target_color = ('red',)
        self.color_sequence = ['red', 'blue', 'green']
        self.color_index = 0
        self.roi = ()
        self.rect = None
        self.rotation_angle = 0
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.last_x, self.last_y = 0, 0
        self.t1 = 0
        self.count = 0
        self.center_list = []
        self.get_roi = False
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

    def reset(self):
        self.count = 0
        self.get_roi = False
        self.center_list = []
        self.color_index = 0
        self.target_color = (self.color_sequence[self.color_index],)
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.rotation_angle = 0
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.last_x, self.last_y = 0, 0
        self.t1 = 0
        self.roi = ()
        self.rect = None

    def set_target_color(self, target_color):
        self.target_color = target_color
        return (True, ())

    def advance_target_color(self):
        self.color_index += 1
        if self.color_index < len(self.color_sequence):
            self.target_color = (self.color_sequence[self.color_index],)
            return True
        return False

    def preprocess_frame(self, img):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)
        return cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    def find_target_contour(self, frame_lab):
        area_max = 0
        area_max_contour = None
        for color in color_range:
            if color in self.target_color:
                self.detect_color = color
                frame_mask = cv2.inRange(frame_lab, color_range[self.detect_color][0], color_range[self.detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                area_max_contour, area_max = get_area_max_contour(contours)
        return area_max_contour, area_max

    def update_stability_state(self):
        distance = math.sqrt(pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))
        self.last_x, self.last_y = self.world_x, self.world_y
        if not self.action_finish:
            return
        if distance < 0.3:
            self.center_list.extend((self.world_x, self.world_y))
            self.count += 1
            if self.start_count_t1:
                self.start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1.5:
                self.rotation_angle = self.rect[2]
                self.start_count_t1 = True
                self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2), axis=0)
                self.count = 0
                self.center_list = []
                self.start_pick_up = True
        else:
            self.t1 = time.time()
            self.start_count_t1 = True
            self.count = 0
            self.center_list = []

    def handle_detected_object(self, img, area_max_contour):
        self.rect = cv2.minAreaRect(area_max_contour)
        box = np.int0(cv2.boxPoints(self.rect))
        self.roi = getROI(box)
        self.get_roi = True
        img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, square_length)
        self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size)
        cv2.drawContours(img, [box], -1, RANGE_RGB[self.detect_color], 2)
        cv2.putText(
            img,
            '(' + str(self.world_x) + ',' + str(self.world_y) + ')',
            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            RANGE_RGB[self.detect_color],
            1
        )
        self.update_stability_state()

    def run(self, img, is_running):
        if not is_running:
            return img
        frame_lab = self.preprocess_frame(img)
        if self.start_pick_up:
            return img
        area_max_contour, area_max = self.find_target_contour(frame_lab)
        if area_max > 2500 and area_max_contour is not None:
            self.handle_detected_object(img, area_max_contour)
        return img


class Motion:
    def __init__(self, perception):
        self.perception = perception
        self.servo1 = 600
        self.operation_mode = 'stack'
        self.is_running = False
        self.stop_requested = False
        self.unreachable = False
        self.sort_coordinate = {
            'red': (-14.5, 11.5, 1.5),
            'green': (-14.5, 5.5, 1.5),
            'blue': (-14.5, -0.5, 1.5),
        }
        self.stack_coordinate = {
            'red': (-14, -7.5, 1.5),
            'green': (-14, -7.5, 1.5),
            'blue': (-14, -7.5, 1.5),
        }
        self.z_r = self.stack_coordinate['red'][2]
        self.z_g = self.stack_coordinate['green'][2]
        self.z_b = self.stack_coordinate['blue'][2]
        self.z = self.z_r
        self.thread = threading.Thread(target=self.move_loop, daemon=True)
        self.thread.start()

    def reset(self):
        self.stop_requested = False
        self.unreachable = False
        self.z_r = self.stack_coordinate['red'][2]
        self.z_g = self.stack_coordinate['green'][2]
        self.z_b = self.stack_coordinate['blue'][2]
        self.z = self.z_r
        self.perception.reset()

    def set_mode(self, mode):
        mode = str(mode).lower()
        self.operation_mode = 'sort' if mode == 'sort' else 'stack'
        print('motion_perception mode:', self.operation_mode)
        return (True, ())

    def init_move(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    @staticmethod
    def set_buzzer(timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    @staticmethod
    def set_rgb(color):
        if color == 'red':
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        elif color == 'green':
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        elif color == 'blue':
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

    def init(self):
        print('ColorTracking Init')
        self.init_move()

    def start(self):
        self.reset()
        self.is_running = True
        print('ColorTracking Start')

    def stop(self):
        self.stop_requested = True
        self.is_running = False
        print('ColorTracking Stop')

    def exit(self):
        self.stop_requested = True
        self.is_running = False
        print('ColorTracking Exit')

    def execute_pick_and_place(self):
        detect_color = self.perception.detect_color
        self.perception.action_finish = False
        self.set_rgb(detect_color)
        self.set_buzzer(0.1)

        if self.operation_mode == 'sort':
            target = self.sort_coordinate[detect_color]
            self.z = target[2]
        else:
            target = self.stack_coordinate[detect_color]
            self.z = self.z_r
            self.z_r += 2.5
            if self.z == 5.0 + self.stack_coordinate['red'][2]:
                self.z_r = self.stack_coordinate['red'][2]

        result = AK.setPitchRangeMoving((self.perception.world_X, self.perception.world_Y, 7), -90, -90, 0)
        if result is False:
            self.unreachable = True
            self.perception.start_pick_up = False
            self.perception.detect_color = 'None'
            self.perception.action_finish = True
            return

        self.unreachable = False
        time.sleep(result[2] / 1000)
        if not self.is_running:
            self.perception.action_finish = True
            return

        servo2_angle = getAngle(self.perception.world_X, self.perception.world_Y, self.perception.rotation_angle)
        Board.setBusServoPulse(1, self.servo1 - 280, 500)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)
        if not self.is_running:
            self.perception.action_finish = True
            return

        AK.setPitchRangeMoving((self.perception.world_X, self.perception.world_Y, 2), -90, -90, 0, 1000)
        time.sleep(1.5)
        if not self.is_running:
            self.perception.action_finish = True
            return

        Board.setBusServoPulse(1, self.servo1, 500)
        time.sleep(0.8)
        if not self.is_running:
            self.perception.action_finish = True
            return

        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((self.perception.world_X, self.perception.world_Y, 12), -90, -90, 0, 1000)
        time.sleep(1)
        if not self.is_running:
            self.perception.action_finish = True
            return

        AK.setPitchRangeMoving((target[0], target[1], 12), -90, -90, 0, 1500)
        time.sleep(1.5)
        if not self.is_running:
            self.perception.action_finish = True
            return

        servo2_angle = getAngle(target[0], target[1], -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)
        if not self.is_running:
            self.perception.action_finish = True
            return

        AK.setPitchRangeMoving((target[0], target[1], self.z + 3), -90, -90, 0, 500)
        time.sleep(0.5)
        if not self.is_running:
            self.perception.action_finish = True
            return

        AK.setPitchRangeMoving((target[0], target[1], self.z), -90, -90, 0, 1000)
        time.sleep(0.8)
        if not self.is_running:
            self.perception.action_finish = True
            return

        Board.setBusServoPulse(1, self.servo1 - 200, 500)
        time.sleep(1)
        if not self.is_running:
            self.perception.action_finish = True
            return

        AK.setPitchRangeMoving((target[0], target[1], 12), -90, -90, 0, 800)
        time.sleep(0.8)
        self.init_move()
        time.sleep(1.5)

        self.perception.detect_color = 'None'
        self.perception.get_roi = False
        self.perception.start_pick_up = False
        self.perception.action_finish = True
        self.set_rgb(self.perception.detect_color)

        if not self.perception.advance_target_color():
            self.is_running = False

    def move_loop(self):
        while True:
            if self.is_running:
                if self.perception.detect_color != 'None' and self.perception.start_pick_up:
                    self.execute_pick_and_place()
                else:
                    time.sleep(0.01)
            else:
                if self.stop_requested:
                    self.stop_requested = False
                    Board.setBusServoPulse(1, self.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)


perception = Perception()
motion = Motion(perception)


def setTargetColor(target_color):
    return perception.set_target_color(target_color)


def setMode(mode):
    return motion.set_mode(mode)


def init():
    motion.init()


def start():
    motion.start()


def stop():
    motion.stop()


def exit():
    motion.exit()


def run(img):
    return perception.run(img, motion.is_running)


if __name__ == '__main__':
    init()
    start()
    perception.target_color = ('red',)
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame_out = run(frame)
            cv2.imshow('Frame', frame_out)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
