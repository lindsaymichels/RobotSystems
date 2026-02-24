#!/usr/bin/env python3
# encoding:utf-8
import os
import cv2
import glob
import time
from CalibrationConfig import *

#采集标定图像，保存在calib文件夹下
#按下键盘上的space键存储图像，按esc退出

cap = cv2.VideoCapture(-1)

#如果calib文件夹不存在，则新建
if not os.path.exists(save_path):
    os.makedirs(save_path, exist_ok=True)

#计算存储的图片数量（避免覆盖已有图片）
existing = glob.glob(save_path + "*.jpg")
num = len(existing)

print("CollectCalibrationPicture")
print("Controls: Space=snapshot, Esc=quit")
print("Saving to:", save_path)
while True:
    ret, frame = cap.read()
    if not ret:
        time.sleep(0.01)
        continue

    Frame = frame.copy()
    cv2.putText(Frame, "count: " + str(num), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    cv2.putText(Frame, "Space: save  Esc: exit", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.imshow("Frame", Frame)

    key = cv2.waitKey(10) & 0xFF
    if key == 27:
        print("Exit")
        break
    if key == ord(' '):
        num += 1
        filename = save_path + str(num) + ".jpg"
        ok = cv2.imwrite(filename, frame)
        print("Saved:" if ok else "Failed:", filename)

cap.release()
cv2.destroyAllWindows()
