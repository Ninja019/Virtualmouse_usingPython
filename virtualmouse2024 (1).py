# pip install opencv-contrib-python
# pip install cvzone
# pip install mouse
# pip install pyautogui
import cv2
from cvzone.HandTrackingModule import HandDetector
import mouse
import numpy as np
import pyautogui
import threading
import time

detector = HandDetector(detectionCon=0.9, maxHands=1)

cap = cv2.VideoCapture(0)
cam_w, cam_h = 640, 480
cap.set(3, cam_w)
cap.set(4, cam_h)

frameR = 100

screen_w, screen_h = pyautogui.size()

while True:
    success, img = cap.read()
    if not success:
        print("Failed to capture image")
        break

    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)
    cv2.rectangle(img,(frameR,frameR),(cam_w - frameR, cam_h - frameR),(255,0,255),2)

    if hands:
        lmlist = hands[0]['lmList']
        ind_x, ind_y = lmlist[8][0], lmlist[8][1]
        mid_x, mid_y = lmlist[12][0], lmlist[12][1]
        cv2.circle(img, (ind_x, ind_y), 5, (0, 255, 255), 2)
        fingers = detector.fingersUp(hands[0])
        print(fingers)


         # Mouse movement
        if fingers[1] == 1 and fingers[2] == 0 and fingers[0] == 1:
            conv_x = int(np.interp(ind_x, (frameR, cam_w - frameR), (0, screen_w)))
            conv_y = int(np.interp(ind_y, (frameR, cam_h - frameR), (0, screen_h)))
        # Move the mouse to the converted coordinates
            mouse.move(conv_x, conv_y)
            print(f'Moving mouse to ({conv_x}, {conv_y})')

        # Mouse button Click
        if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 1:
            if abs(ind_x-mid_x) < 25 :
                mouse.click(button="left")

    cv2.imshow("Camera Feed", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


# pip install opencv-contrib-python
# pip install cvzone
# pip install mouse
# pip install pyautogui
import cv2
from cvzone.HandTrackingModule import HandDetector
import mouse
import numpy as np
import pyautogui

detector = HandDetector(detectionCon=0.9, maxHands=1)

cap = cv2.VideoCapture(0)
cam_w, cam_h = 640, 480
cap.set(3, cam_w)
cap.set(4, cam_h)

frameR = 100

screen_w, screen_h = pyautogui.size()

while True:
    success, img = cap.read()
    if not success:
        print("Failed to capture image")
        break

    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)
    cv2.rectangle(img,(frameR,frameR),(cam_w - frameR, cam_h - frameR),(255,0,255),2)

    if hands:
        lmlist = hands[0]['lmList']
        ind_x, ind_y = lmlist[8][0], lmlist[8][1]
        mid_x, mid_y = lmlist[12][0], lmlist[12][1]
        cv2.circle(img, (ind_x, ind_y), 5, (0, 255, 255), 2)
        fingers = detector.fingersUp(hands[0])
        print(fingers)

        # Mouse movement
        if fingers[1] == 1 and fingers[2] == 0 and fingers[0] == 1:
            conv_x = int(np.interp(ind_x, (frameR, cam_w - frameR), (0, screen_w)))
            conv_y = int(np.interp(ind_y, (frameR, cam_h - frameR), (0, screen_h)))
            # Move the mouse to the converted coordinates
            mouse.move(conv_x, conv_y)
            print(f'Moving mouse to ({conv_x}, {conv_y})')

        # Left click
        if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 1:
            if abs(ind_x - mid_x) < 25:
                mouse.click(button="left")

        # Right click
        if fingers[1] == 0 and fingers[2] == 1 and fingers[0] == 1:
            if abs(ind_x - mid_x) < 25:
                mouse.click(button="right")

    cv2.imshow("Camera Feed", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


# pip install opencv-contrib-python
# pip install cvzone
# pip install mouse
# pip install pyautogui
import cv2
from cvzone.HandTrackingModule import HandDetector
import mouse
import numpy as np
import pyautogui

detector = HandDetector(detectionCon=0.9, maxHands=1)

cap = cv2.VideoCapture(0)
cam_w, cam_h = 640, 480
cap.set(3, cam_w)
cap.set(4, cam_h)

frameR = 100

screen_w, screen_h = pyautogui.size()

while True:
    success, img = cap.read()
    if not success:
        print("Failed to capture image")
        break

    img = cv2.flip(img, 1)
    hands, img = detector.findHands(img, flipType=False)
    cv2.rectangle(img, (frameR, frameR), (cam_w - frameR, cam_h - frameR), (255, 0, 255), 2)

    if hands:
        lmlist = hands[0]['lmList']
        ind_x, ind_y = lmlist[8][0], lmlist[8][1]
        mid_x, mid_y = lmlist[12][0], lmlist[12][1]
        cv2.circle(img, (ind_x, ind_y), 5, (0, 255, 255), 2)
        fingers = detector.fingersUp(hands[0])
        print(fingers)

        # Mouse movement
        if fingers[1] == 1 and fingers[2] == 0 and fingers[0] == 1:
            conv_x = int(np.interp(ind_x, (frameR, cam_w - frameR), (0, screen_w)))
            conv_y = int(np.interp(ind_y, (frameR, cam_h - frameR), (0, screen_h)))
            # Move the mouse to the converted coordinates
            mouse.move(conv_x, conv_y)
            print(f'Moving mouse to ({conv_x}, {conv_y})')

        # Left click
        if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 1:
            if abs(ind_x - mid_x) < 25:
                mouse.click(button="left")
                print("Left Click")

        # Right click
        if fingers[1] == 1 and fingers[2] == 1 and fingers[0] == 0:
            if abs(ind_x - mid_x) < 20:
                mouse.click(button="right")
                print("Right Click")

    cv2.imshow("Camera Feed", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()