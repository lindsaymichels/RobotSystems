#!/usr/bin/python3
# coding=utf8
import sys
from pathlib import Path
BASE_DIR = Path(__file__).resolve().parents[1]
if str(BASE_DIR) not in sys.path:
    sys.path.append(str(BASE_DIR))
import cv2
import time
import math
import Camera
import threading
import numpy as np
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Gripper closed angle used for grasping
servo1 = 500

# Runtime state
count = 0
track = False
_stop = False
get_roi = False
center_list = []
first_move = True
__isRunning = False
detect_color = 'None'
action_finish = True
start_pick_up = False
start_count_t1 = True

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0

__target_color = ('red',)


def setTargetColor(target_color):
    global __target_color
    __target_color = target_color
    perception.target_color = target_color
    return (True, ())


# Find the contour with the largest area.
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:
                area_max_contour = c

    return area_max_contour, contour_area_max


class Motion:
    def __init__(self, arm_ik):
        self.ak = arm_ik
        self.is_running = False
        # Coordinates for placing different colored wooden blocks(x, y, z)
        self.coordinate = {
            'red': (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5, 1.5),
            'blue': (-15 + 0.5, 0 - 0.5, 1.5),
        }

    def init_move(self):
        Board.setBusServoPulse(1, servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.ak.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def set_buzzer(self, timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    def set_rgb(self, color):
        if color == 'red':
            rgb = Board.PixelColor(255, 0, 0)
        elif color == 'green':
            rgb = Board.PixelColor(0, 255, 0)
        elif color == 'blue':
            rgb = Board.PixelColor(0, 0, 255)
        else:
            rgb = Board.PixelColor(0, 0, 0)
        Board.RGB.setPixelColor(0, rgb)
        Board.RGB.setPixelColor(1, rgb)
        Board.RGB.show()

    def move(self):
        global rect
        global track
        global _stop
        global get_roi
        global unreachable
        global detect_color
        global action_finish
        global rotation_angle
        global world_X, world_Y
        global world_x, world_y
        global center_list, count
        global start_pick_up, first_move

        while True:
            if self.is_running:
                if first_move and start_pick_up:
                    action_finish = False
                    self.set_rgb(detect_color)
                    self.set_buzzer(0.1)
                    result = self.ak.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0)
                    if result is False:
                        unreachable = True
                    else:
                        unreachable = False
                        time.sleep(result[2] / 1000)
                    start_pick_up = False
                    first_move = False
                    action_finish = True
                elif not first_move and not unreachable:
                    self.set_rgb(detect_color)
                    if track:
                        if not self.is_running:
                            continue
                        self.ak.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)
                        track = False
                    if start_pick_up:
                        action_finish = False
                        if not self.is_running:
                            continue
                        Board.setBusServoPulse(1, servo1 - 280, 500)
                        servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)

                        if not self.is_running:
                            continue
                        self.ak.setPitchRangeMoving((world_X, world_Y, 1.5), -90, -90, 0, 1000)
                        time.sleep(2)

                        if not self.is_running:
                            continue
                        Board.setBusServoPulse(1, servo1, 500)
                        time.sleep(1)

                        if not self.is_running:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        self.ak.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)
                        time.sleep(1)

                        if not self.is_running:
                            continue
                        result = self.ak.setPitchRangeMoving(
                            (self.coordinate[detect_color][0], self.coordinate[detect_color][1], 12),
                            -90, -90, 0,
                        )
                        time.sleep(result[2] / 1000)

                        if not self.is_running:
                            continue
                        servo2_angle = getAngle(
                            self.coordinate[detect_color][0], self.coordinate[detect_color][1], -90
                        )
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.is_running:
                            continue
                        self.ak.setPitchRangeMoving(
                            (
                                self.coordinate[detect_color][0],
                                self.coordinate[detect_color][1],
                                self.coordinate[detect_color][2] + 3,
                            ),
                            -90,
                            -90,
                            0,
                            500,
                        )
                        time.sleep(0.5)

                        if not self.is_running:
                            continue
                        self.ak.setPitchRangeMoving((self.coordinate[detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)

                        if not self.is_running:
                            continue
                        Board.setBusServoPulse(1, servo1 - 200, 500)
                        time.sleep(0.8)

                        if not self.is_running:
                            continue
                        self.ak.setPitchRangeMoving(
                            (self.coordinate[detect_color][0], self.coordinate[detect_color][1], 12),
                            -90,
                            -90,
                            0,
                            800,
                        )
                        time.sleep(0.8)

                        self.init_move()
                        time.sleep(1.5)

                        detect_color = 'None'
                        first_move = True
                        get_roi = False
                        action_finish = True
                        start_pick_up = False
                        self.set_rgb(detect_color)
                else:
                    time.sleep(0.01)
            else:
                if _stop:
                    _stop = False
                    Board.setBusServoPulse(1, servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    self.ak.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)


class Perception:
    def __init__(self, tracking_size):
        self.size = tracking_size
        self.target_color = ('red',)
        self.t1 = 0
        self.roi = ()
        self.last_x = 0
        self.last_y = 0
        self.is_running = False

    def preprocess_frame(self, img):
        global get_roi
        global start_pick_up

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        # If an identified object is in a certain area, keep focusing that ROI.
        if get_roi and start_pick_up:
            get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)

        return cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

    def find_target_contour(self, frame_lab):
        global detect_color

        area_max = 0
        areaMaxContour = None
        for color in color_range:
            if color in self.target_color:
                detect_color = color
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                areaMaxContour, area_max = getAreaMaxContour(contours)

        return areaMaxContour, area_max

    def update_stability_state(self, current_rect):
        global count
        global track
        global center_list
        global action_finish
        global rotation_angle
        global world_X, world_Y
        global world_x, world_y
        global start_count_t1
        global start_pick_up

        distance = math.sqrt(pow(world_x - self.last_x, 2) + pow(world_y - self.last_y, 2))
        self.last_x, self.last_y = world_x, world_y
        track = True

        if not action_finish:
            return

        if distance < 0.3:
            center_list.extend((world_x, world_y))
            count += 1
            if start_count_t1:
                start_count_t1 = False
                self.t1 = time.time()
            if time.time() - self.t1 > 1.5:
                rotation_angle = current_rect[2]
                start_count_t1 = True
                world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                count = 0
                center_list = []
                start_pick_up = True
        else:
            self.t1 = time.time()
            start_count_t1 = True
            count = 0
            center_list = []

    def handle_detected_object(self, img, areaMaxContour):
        global rect
        global get_roi
        global world_x, world_y

        rect = cv2.minAreaRect(areaMaxContour)
        box = np.int0(cv2.boxPoints(rect))

        self.roi = getROI(box)
        get_roi = True

        img_centerx, img_centery = getCenter(rect, self.roi, self.size, square_length)
        world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)

        cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
        cv2.putText(
            img,
            '(' + str(world_x) + ',' + str(world_y) + ')',
            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            range_rgb[detect_color],
            1,
        )
        self.update_stability_state(rect)

    def run(self, img):
        global __isRunning
        global start_pick_up

        if not self.is_running:
            return img

        frame_lab = self.preprocess_frame(img)
        if start_pick_up:
            return img

        areaMaxContour, area_max = self.find_target_contour(frame_lab)
        if area_max > 2500 and areaMaxContour is not None:
            self.handle_detected_object(img, areaMaxContour)

        return img


motion = Motion(AK)
perception = Perception(size)


def initMove():
    motion.init_move()


def setBuzzer(timer):
    motion.set_buzzer(timer)


def set_rgb(color):
    motion.set_rgb(color)


def reset():
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True
    motion.is_running = False
    perception.is_running = False
    perception.target_color = ()
    perception.t1 = 0
    perception.roi = ()
    perception.last_x = 0
    perception.last_y = 0


def init():
    print('ColorTracking Init')
    initMove()


def start():
    global __isRunning
    reset()
    __isRunning = True
    motion.is_running = True
    perception.is_running = True
    print('ColorTracking Start')


def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    motion.is_running = False
    perception.is_running = False
    print('ColorTracking Stop')


def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    motion.is_running = False
    perception.is_running = False
    print('ColorTracking Exit')


# Run motion thread
th = threading.Thread(target=motion.move)
th.setDaemon(True)
th.start()


def run(img):
    return perception.run(img)


if __name__ == '__main__':
    init()
    start()
    setTargetColor(('red',))
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
