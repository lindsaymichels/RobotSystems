#!/usr/bin/python3
# coding=utf8
import sys
import os
from pathlib import Path
import importlib.util
BASE_DIR = Path(__file__).resolve().parent
if str(BASE_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_DIR))
import cv2
import time
import queue
import logging
import threading
import RPCServer as RPCServer
import MjpgServer as MjpgServer
import HiwonderSDK.Board as Board
import Running as Running

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

QUEUE_RPC = queue.Queue(10)
SORT_FUNC_ID = 3
STACK_FUNC_ID = 4

def load_camera_module():
    for module_name in ("Camera", "camera"):
        try:
            return __import__(module_name)
        except ModuleNotFoundError:
            pass

    candidates = [
        BASE_DIR / "Camera.py",
        BASE_DIR / "camera.py",
        Path("/home/pi/ArmPi/Camera.py"),
        Path("/home/pi/RobotSystems/ArmPi/Camera.py"),
    ]
    for cam_path in candidates:
        if not cam_path.exists():
            continue
        spec = importlib.util.spec_from_file_location("Camera", str(cam_path))
        if spec and spec.loader:
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module

    raise ModuleNotFoundError("Camera module not found. Expected Camera.py in ArmPi directory.")

def switch_mode(func_id, target_colors=('red', 'green', 'blue')):
    if Running.RunningFunc != 0:
        Running.stopFunc(())
    Running.loadFunc((func_id,))
    exe = Running.CurrentEXE()
    if hasattr(exe, 'setTargetColor'):
        exe.setTargetColor(target_colors)
    Running.startFunc(())
    print("Switched mode to", "ColorSorting" if func_id == SORT_FUNC_ID else "ColorPalletizing")

def switch_to_sort(target_colors=('red', 'green', 'blue')):
    switch_mode(SORT_FUNC_ID, target_colors)

def switch_to_stack(target_colors=('red', 'green', 'blue')):
    switch_mode(STACK_FUNC_ID, target_colors)

def _stdin_switch_task():
    print("Mode switch commands: 'sort', 'stack', 'status'")
    while True:
        try:
            cmd = input().strip().lower()
        except EOFError:
            break
        except Exception:
            time.sleep(0.1)
            continue
        if cmd == 'sort':
            switch_to_sort()
        elif cmd == 'stack':
            switch_to_stack()
        elif cmd == 'status':
            print("RunningFunc =", Running.RunningFunc)
        elif cmd:
            print("Unknown command:", cmd)

def apply_startup_mode_from_argv():
    if len(sys.argv) < 2:
        return
    mode = sys.argv[1].strip().lower()
    if mode == 'sort':
        switch_to_sort()
    elif mode == 'stack':
        switch_to_stack()
    else:
        print("Unknown startup mode:", mode, "(use 'sort' or 'stack')")

def startArmPi():
    global HWEXT, HWSONIC

    RPCServer.QUEUE = QUEUE_RPC

    threading.Thread(target=RPCServer.startRPCServer,
                     daemon=True).start()  # rpc服务器
    threading.Thread(target=MjpgServer.startMjpgServer,
                     daemon=True).start()  # mjpg流服务器
    threading.Thread(target=_stdin_switch_task,
                     daemon=True).start()  # local stdin mode switch
    
    loading_picture = cv2.imread(str(BASE_DIR / 'CameraCalibration' / 'loading.jpg'))
    CameraModule = load_camera_module()
    cam = CameraModule.Camera()  # 相机读取
    Running.cam = cam
    apply_startup_mode_from_argv()

    while True:
        time.sleep(0.03)

        # 执行需要在本线程中执行的RPC命令
        while True:
            try:
                req, ret = QUEUE_RPC.get(False)
                event, params, *_ = ret
                ret[2] = req(params)  # 执行RPC命令
                event.set()
            except:
                break
        #####
        # 执行功能玩法程序：
        try:
            if Running.RunningFunc > 0 and Running.RunningFunc <= 6:
                if cam.frame is not None:
                    MjpgServer.img_show = Running.CurrentEXE().run(cam.frame.copy())
                else:
                    MjpgServer.img_show = loading_picture
            else:
                cam.frame = None
        except KeyboardInterrupt:
            break

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
    startArmPi()
