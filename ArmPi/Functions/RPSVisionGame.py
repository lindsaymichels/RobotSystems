#!/usr/bin/python3
# coding=utf8
import sys
import time
import random
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parents[1]
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))

import Camera

if sys.version_info.major == 2:
    print("Please run this program with python3!")
    sys.exit(0)

VALID_MOVES = ("rock", "paper", "scissors")

__isRunning = False
_stop = False


def init():
    print("RPSVisionGame Init")


def start():
    global __isRunning, _stop
    _stop = False
    __isRunning = True
    print("RPSVisionGame Start")


def stop():
    global __isRunning, _stop
    _stop = True
    __isRunning = False
    print("RPSVisionGame Stop")


def exit():
    global __isRunning, _stop
    _stop = True
    __isRunning = False
    print("RPSVisionGame Exit")


def run(img):
    # Keep camera stream passthrough compatible with existing framework.
    return img


def detect_human_move(frame):
    """
    TODO: Replace this with your friend's OpenCV logic.
    Must return one of:
      "rock", "paper", "scissors", or "unknown"
    """
    _ = frame
    return "unknown"


def choose_robot_move():
    return random.choice(VALID_MOVES)


def decide_winner(human_move, robot_move):
    if human_move not in VALID_MOVES:
        return "unknown"
    if human_move == robot_move:
        return "tie"
    wins = {
        ("rock", "scissors"),
        ("paper", "rock"),
        ("scissors", "paper"),
    }
    return "human" if (human_move, robot_move) in wins else "robot"


def play_one_round(camera):
    frame = camera.frame
    if frame is None:
        return None

    human_move = detect_human_move(frame)
    robot_move = choose_robot_move()
    winner = decide_winner(human_move, robot_move)
    return {
        "human_move": human_move,
        "robot_move": robot_move,
        "winner": winner,
    }


if __name__ == "__main__":
    init()
    start()

    cam = Camera.Camera()
    cam.camera_open()
    print("RPS ready. Ctrl+C to stop.")

    try:
        while True:
            if __isRunning:
                result = play_one_round(cam)
                if result is not None:
                    print(result)
                time.sleep(1.0)
            else:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        cam.camera_close()
