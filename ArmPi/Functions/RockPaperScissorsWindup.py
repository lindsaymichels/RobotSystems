#!/usr/bin/python3
# coding=utf8
import sys
import time
import threading
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[1]
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

from ArmIK.ArmMoveIK import *  # noqa: F401,F403
import HiwonderSDK.Board as Board

if sys.version_info.major == 2:
    print("Please run this program with python3!")
    sys.exit(0)

AK = ArmIK()
servo1 = 500
__isRunning = False
_busy = False
_stop = False

# Neutral pose and motion tuning
BASE_XY = (0, 12)
UP_Z = 12
DOWN_Z = 5
WINDUP_CYCLES = 3


def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1200)


def reset():
    global _stop, _busy
    _stop = False
    _busy = False


def init():
    print("RockPaperScissorsWindup Init")
    initMove()


def start():
    global __isRunning
    reset()
    __isRunning = True
    print("RockPaperScissorsWindup Start")


def stop():
    global __isRunning, _stop
    _stop = True
    __isRunning = False
    print("RockPaperScissorsWindup Stop")


def exit():
    global __isRunning, _stop
    _stop = True
    __isRunning = False
    print("RockPaperScissorsWindup Exit")


def run(img):
    return img


def _windup_once():
    global _busy
    if _busy:
        return
    _busy = True
    try:
        for _ in range(WINDUP_CYCLES):
            if _stop or not __isRunning:
                break
            AK.setPitchRangeMoving((BASE_XY[0], BASE_XY[1], UP_Z), -90, -90, 0, 350)
            time.sleep(0.4)
            if _stop or not __isRunning:
                break
            AK.setPitchRangeMoving((BASE_XY[0], BASE_XY[1], DOWN_Z), -90, -90, 0, 350)
            time.sleep(0.4)
        initMove()
    finally:
        _busy = False


def _move_thread():
    while True:
        if __isRunning and not _busy:
            _windup_once()
            __isRunning_local_sleep = 0.05
        else:
            __isRunning_local_sleep = 0.01
        time.sleep(__isRunning_local_sleep)


th = threading.Thread(target=_move_thread, daemon=True)
th.start()


if __name__ == "__main__":
    init()
    start()
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop()
