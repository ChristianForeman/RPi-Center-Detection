#!/usr/bin/env python3
from picamera import PiCamera
from time import sleep
import time
import serial

import numpy as np
import cv2
import imutils

#Globals:
camera = PiCamera()

# TODO: Tune these values to be more precise
boundary = ([0,0,100], [75,75,225])

# Set up the UART port
ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1000
)

# Take a photo, save as current img
def takePhoto():
    camera.capture("currentImg.jpg")
    return cv2.imread("currentImg.jpg")

# Calculate the midpoint between points
def midpoint(pA, pB):
    return ((pA[0] + pB[0]) * 0.5, (pA[0] + pB[0]) * 0.5)

def detectImg(image, detectionPath, midpointPath):
    lower = np.array(boundary[0], dtype="uint8")
    upper = np.array(boundary[1], dtype="uint8")

    # select pixels that are in the boundary
    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)

    cv2.imwrite(detectionPath, np.hstack([image,output]))

    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    orig = image.copy()

    total_x = 0
    total_y = 0
    totalContours = 0

    for c in cnts:
        if cv2.contourArea(c) < 100:
            continue
        
        # orig = image.copy()
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")

        # box = imutils.perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0, 2))

        x_mid = 0
        y_mid = 0
        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 5, (0, 255, 0), -1)
            x_mid += x
            y_mid += y
        
        x_mid = x_mid / 4
        y_mid = y_mid / 4
        total_x += x_mid
        total_y += y_mid
        totalContours = totalContours + 1
        cv2.circle(orig, (int(x_mid), int(y_mid)), 5, (0, 255, 0), -1)

    total_x = total_x / 4
    total_y = total_y / 4

    success = True
    # If there are not 4 contours, we did not detect 4 corners
    if totalContours != 4:
        success = False
        return success, 0, 0

    # there should be a check that we saw exactly 4 midpoints (contours), if not, we must send an invalid photo message

    cv2.circle(orig, (int(total_x), int(total_y)), 5, (0, 255, 0), -1)

    cv2.circle(orig, (int(image.shape[1] / 2), int(image.shape[0] / 2)), 5, (255, 0, 0), -1)

    cv2.line(orig, (int(total_x), int(total_y)), (int(image.shape[1] / 2), int(image.shape[0] / 2)), (0, 0, 255), 3)

    cv2.imwrite(midpointPath, orig)

    offset_x = int(total_x - image.shape[1] / 2)
    offset_y = int(total_y - image.shape[0] / 2)

    return success, offset_x, offset_y

def uart_to_nucleo(msg):
    ser.write(str(msg).encode())
    time.sleep(0.5)

def main():
    print("Starting up")

    msg = None

    for x in range(25):
        print("taking photo", str(x))
        image = takePhoto()
        imgName = "img" + str(x) + ".jpg"
        success, offset_x, offset_y = detectImg(image, "detections/" + imgName, "midpoints/" + imgName)

        if success == True:
            # Add the bytes for the horizontal component
            if offset_x < 0:
                msg = "l"
                offset_x = abs(offset_x)
            else:
                msg = "r"
            
            offset_string = str(offset_x)
            msg = msg + "0" * (3 - len(offset_string))
            msg = msg + offset_string

            # Now work on the vertical portion
            if offset_y < 0:
                msg = msg + "u"
                offset_y = abs(offset_y)
            else:
                msg = msg + "d"
            
            offset_string = str(offset_y)
            msg = msg + "0" * (3 - len(offset_string))
            msg = msg + offset_string

            print("photo", x, "is sending", msg)
            uart_to_nucleo(msg)
        else:
            print("No image detected, not sending a message via UART")

        sleep(1)
    
    camera.close()

    print("Main is terminating")

def test():
    print("Test up")

    image = cv2.imread("testingImages/img0.jpg")
    detectImg(image, "img0.jpg")
    
    print("Test is terminating")


if __name__ == "__main__":
    main()