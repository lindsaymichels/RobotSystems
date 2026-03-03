#!/usr/bin/python3
# coding=utf8
import sys
from pathlib import Path
BASE_DIR = Path(__file__).resolve().parents[1]
if str(BASE_DIR) not in sys.path:
    sys.path.append(str(BASE_DIR))
import cv2
import time
import Camera
import threading
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

__target_color = ('red',)
# set color
def setTargetColor(target_color):
    global __target_color

    #print("COLOR", target_color)
    __target_color = target_color
    return (True, ())

# Find the contour with the largest area
# The parameter is a list of contours to be compared.
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # Traversing all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Compute contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # Only treat contours >300 area as valid to filter noise
                area_max_contour = c

    return area_max_contour, contour_area_max  # Return the largest contour

# Gripper closed angle used for grasping
servo1 = 500

# Initial position
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)

# Set the RGB LEDs to match the tracked color
def set_rgb(color):
    if color == "red":
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
        Board.RGB.show()
    elif color == "green":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
        Board.RGB.show()
    elif color == "blue":
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
        Board.RGB.show()
    else:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()

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
# Reset variables
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

# Called when app initializes
def init():
    print("ColorTracking Init")
    initMove()

# Called when app starts
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")

# Called when app stops
def stop():
    global _stop 
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Stop")

# Called when app exits
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Exit")

rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0
# robotic arm movement thread
def move():
    global rect
    global track
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global action_finish
    global rotation_angle
    global world_X, world_Y
    global world_x, world_y
    global center_list, count
    global start_pick_up, first_move

    while True:
        if __isRunning:
            if first_move and start_pick_up: # When an object is first detected               
                action_finish = False
                set_rgb(detect_color)
                setBuzzer(0.1)               
                result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0) # If no runtime parameter is specified, the runtime will be adaptive
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                time.sleep(result[2]/1000) # The third parameter returned is time.
                start_pick_up = False
                first_move = False
                action_finish = True
                track = True 
            elif not first_move and not unreachable: # This is not the first time an object has been detected.
                set_rgb(detect_color)
                if track: # if tracking stage
                    if not __isRunning: # Stop and exit flag detection
                        continue
                    AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
                    time.sleep(0.02)
                    track = False
                else:
                    time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)
# Run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

t1 = 0
roi = ()
last_x, last_y = 0, 0

def preprocess_frame(img):
    global get_roi
    global start_pick_up

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

    # If an identified object is detected in a certain area, continue
    # detecting that area until no object is detected.
    if get_roi and start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)

    return cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

def find_target_contour(frame_lab):
    global detect_color

    area_max = 0
    areaMaxContour = None
    for color in color_range:
        if color in __target_color:
            detect_color = color
            frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
            contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            areaMaxContour, area_max = getAreaMaxContour(contours)

    return areaMaxContour, area_max

def update_stability_state(rect):
    global count
    global track
    global center_list
    global action_finish
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global world_x, world_y
    global start_count_t1, t1
    global start_pick_up

    distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2))
    last_x, last_y = world_x, world_y
    track = True

    if not action_finish:
        return

    if distance < 0.3:
        center_list.extend((world_x, world_y))
        count += 1
        if start_count_t1:
            start_count_t1 = False
            t1 = time.time()
        if time.time() - t1 > 1.5:
            rotation_angle = rect[2]
            start_count_t1 = True
            world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
            count = 0
            center_list = []
            start_pick_up = True
    else:
        t1 = time.time()
        start_count_t1 = True
        count = 0
        center_list = []

def handle_detected_object(img, areaMaxContour):
    global roi
    global rect
    global get_roi
    global world_x, world_y

    rect = cv2.minAreaRect(areaMaxContour)
    box = np.int0(cv2.boxPoints(rect))

    roi = getROI(box)
    get_roi = True

    img_centerx, img_centery = getCenter(rect, roi, size, square_length)
    world_x, world_y = convertCoordinate(img_centerx, img_centery, size)

    cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
    cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')',
                (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)
    update_stability_state(rect)

def run(img):
    global __isRunning
    global start_pick_up

    if not __isRunning:
        return img

    frame_lab = preprocess_frame(img)
    if start_pick_up:
        return img

    areaMaxContour, area_max = find_target_contour(frame_lab)
    if area_max > 2500 and areaMaxContour is not None:
        handle_detected_object(img, areaMaxContour)

    return img

if __name__ == '__main__':
    init()
    start()
    __target_color = ('red', )
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
