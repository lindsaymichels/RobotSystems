#!/usr/bin/python3
# coding=utf8

import cv2
import numpy as np
import math


def count_fingers(contour, drawing):

    hull = cv2.convexHull(contour, returnPoints=False)
    defects = cv2.convexityDefects(contour, hull)

    if defects is None:
        return 0

    finger_count = 0

    for i in range(defects.shape[0]):
        s, e, f, d = defects[i, 0]

        start = tuple(contour[s][0])
        end = tuple(contour[e][0])
        far = tuple(contour[f][0])

        a = math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)
        b = math.sqrt((start[0] - far[0])**2 + (start[1] - far[1])**2)
        c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
        angle = math.acos((b**2 + c**2 - a**2) / (2*b*c))

        if angle <= math.pi / 2:
            finger_count += 1
            cv2.circle(drawing, far, 5, [0, 0, 255], -1)

    return finger_count


def main():

    cap = cv2.VideoCapture(0)

    print("OpenCV Gesture Recognition Started")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        roi = frame[100:400, 100:400]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Basic skin color range (may need tweaking)
        lower_skin = np.array([0, 30, 60])
        upper_skin = np.array([20, 150, 255])

        mask = cv2.inRange(hsv, lower_skin, upper_skin)

        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        gesture = "No hand"

        if contours:
            contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(contour) > 2000:

                drawing = roi.copy()
                fingers = count_fingers(contour, drawing)

                if fingers == 0:
                    gesture = "FIST"
                elif fingers == 1:
                    gesture = "ONE"
                elif fingers == 2:
                    gesture = "TWO"
                elif fingers == 3:
                    gesture = "THREE"
                elif fingers >= 4:
                    gesture = "OPEN"

                cv2.drawContours(roi, [contour], -1, (0, 255, 0), 2)

        cv2.putText(frame, f"Gesture: {gesture}", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0), 2)

        cv2.rectangle(frame, (100, 100), (400, 400), (255, 0, 0), 2)

        cv2.imshow("Gesture Recognition", frame)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()