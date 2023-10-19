# SEED Lab - Demo 1 - Computer Vission
# Purpose: Determine the angle of an Aruco marker compared to the center of the camera
# Method: Capture an image using the camera. If an Aruco marker is detected, display the andle on
#           the LCD screen. Calculate the angle by determining the distance of the marker from the
#           camera and the left to right distance from the center of the capture. Then take the
#           tangent of a ratio of those values to detemrmine theangle the marker is at.
# Required hardware: LCD screen, camera


# Initialization
import time
import board
import numpy as np
import cv2 as cv
import threading
import queue
import math

from cv2 import aruco
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus
from random import random

# Variable creation
q = queue.Queue()
change = False
ThetaLast = -1

# Disctionary for aruco generation and detection
myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# This code generates markers, uncomment to display a marker
#myAruco = aruco.GridBoard((1,1),4.0,1.0,myDict)
#myArucoImg = myAruco.generateImage((500,500))
#cv.imshow("myArucoImg",myArucoImg)
#cv.imwrite("Aruco.jpg", myArucoImg)

# Capture from the camera plugged into the Pi
capture = cv.VideoCapture(0)
sleep (0.1)

# Set dimensions of the capture and find the center of the window
capWidth = 640
xCapCent = capWidth/2
markWidthCM = 5
capture.set(cv.CAP_PROP_FRAME_WIDTH, capWidth)

si2c = SMBus(1)

# Threading function for LCD screen - improves I2C speed 
def myFunction():
    # LCD screen initialization
    lcd_columns = 16
    lcd_rows = 2
    i2c = board.I2C()
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.color = [50, 0, 50]
    lcd.clear()
    # Loop to find if LCD screen needs to change output
    while True:
        if change:
            lcd.clear
            gotSomething = q.get()
            if(gotSomething != 0):
                ToPrint = str(Theta)
                lcd.message = ToPrint
                lcd.color = [0,150, 50]
            else:
                lcd.color = [255,0,0]
                lcd.message = "\n"
                
myThread = threading.Thread(target=myFunction,args=())
myThread.start()

# Continue this loop until the break at the bottom
while True:

    # Read from the camera capture, set the color to greyscale, run aruco detection
    ret, frame = capture.read()     
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners,ids,rejected = aruco.detectMarkers(gray,myDict)

    # Make an overlay for detection
    overlay = cv.cvtColor(gray, cv.COLOR_GRAY2RGB)
    overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)

    # Enter this code if a marker is found
    if not ids is None:

        # Calculate where the center of the marker is on the screen (x coordinate)
        cornersArray = np.array(corners)[0][0]
        xMarkCent = (cornersArray[0][0] + cornersArray[1][0] + cornersArray[2][0] + cornersArray[3][0]) / 4 

        # Calculate theta
        center2CenterPix = xCapCent - xMarkCent # Center of capture to center of marker in pixels
        markWidthPix = (abs(cornersArray[0][0] - cornersArray[1][0]) + abs(cornersArray[2][0] - cornersArray[3][0]))/2

        distance = 4161.416*(markWidthPix)**(-1.055)    # Marker dstance from camera, centimeters; power interpolation
        left2Right = markWidthCM * center2CenterPix / markWidthPix  # Left to right distance from center, cm
        theta = 90 - ((math.atan2(distance, left2Right)) * 180 / math.pi)     # Angle from center, degrees

        # Round theta to 2 decimal points
        Theta = round(theta, 2)

        # Loop to see if Theta has changed or not
        if (Theta != ThetaLast):
            q.put(Theta)
            change = True
        else:
            change = False
            ThetaLast = Theta
            q.put(0)

        # Edit the overlay display to outline the marker
        ids = ids.flatten()
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
            overlay = cv.putText(overlay, str(id), (int(markerCorners[0,0]),int(markerCorners[0,1])-15),cv.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

    # Show the overlay
    cv.imshow("overlay", overlay)

    # press q key to exit out of the program
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        break

# Once out of while, release the capture and destroy windows
capture.release()
cv.destroyAllWindows()
