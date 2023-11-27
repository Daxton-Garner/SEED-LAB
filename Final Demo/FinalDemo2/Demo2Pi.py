# SEED Lab - Demo 2 - Computer Vission
# Purpose: 
# Method: 
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
stateLast = -1
state = 0
send2Duino = 0
reply = 0
stopDist  = 30.48 * 1.5
loopCount = 0

# BAILEY ADD FOR COMMS
ARD_ADDR = 8
offset = 1
i2c = board.I2C()
i2c = SMBus(1)
# END BAILEY ADD 

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

#Camera tweaks
ret, frame = capture.read()
capture.set(cv.CAP_PROP_AUTO_EXPOSURE, 3)
capture.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)

capture.set(cv.CAP_PROP_BRIGHTNESS, 250)

capture.set(cv.CAP_PROP_EXPOSURE, 39) #5, 9, 10, 19, 20, 39
#End Camera Tweaks

si2c = SMBus(1)

# Threading function for LCD screen - improves I2C speed 
#def LCDThread():
    # LCD screen initialization
    #lcd_columns = 16
    #lcd_rows = 2
    #i2c = board.I2C()
    #lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    #lcd.color = [50, 0, 50]
    #lcd.clear()

    # Loop to find if LCD screen needs to change output
   # while True:
       
        #if change:
        #    gotSomething = q.get()
        #    if(gotSomething != 0) and (state == 1):
        #        #if (theta < 0):
                    #lcd.color = [0, 150, 50]
                #else:
         #       lcd.color = [50, 0, 50]
         #       ToPrint = str(state)
         #       lcd.message = ToPrint
          #  else:
           #     lcd.color = [255,0,0]
           #     lcd.message = "\n"
                
def ArduinoComThread(): 
    while True: 

        if change:
            gotSomething = q.get()
            if(gotSomething != -9):
                try:
                    i2c.write_byte_data(ARD_ADDR, offset, int(send2Duino))  
                except IOError:
                    print("Could not write data toArduino")

    
#myThread = threading.Thread(target=LCDThread,args=())
#myThread.start()

myThreadTwo = threading.Thread(target=ArduinoComThread,args=())
myThreadTwo.start()

# Continue this loop until the break at the bottom
while True:

    # Read from the camera capture, set the color to greyscale, run aruco detection
    ret, frame = capture.read()     
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners,ids,rejected = aruco.detectMarkers(gray,myDict)

    # Make an overlay for detection
    overlay = cv.cvtColor(gray, cv.COLOR_GRAY2RGB)
    overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)

    if state == 0:  # Not searching yet
        state = 0.5
        send2Duino = 1
        reply = 0

    elif state == 0.5:  # Searching but marker isn't seen yet
        if not ids is None:
            state = 1
            send2Duino = 1
        if reply == 8:
            state = 0
            
    elif state == 1:    # Searching but marker is seen
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

            print("Theta: ", round(theta, 2))

            
            if abs(theta) < 15:
                state = 2
                send2Duino = 2
        if reply == 8:
            state = 0


    elif state == 2:    # Marker is in the middle of the screen
        if not ids is None:
            # Calculate where the center of the marker is on the screen (x coordinate)
            cornersArray = np.array(corners)[0][0]
            xMarkCent = (cornersArray[0][0] + cornersArray[1][0] + cornersArray[2][0] + cornersArray[3][0]) / 4 

            # Calculate distance
            center2CenterPix = xCapCent - xMarkCent # Center of capture to center of marker in pixels
            markWidthPix = (abs(cornersArray[0][0] - cornersArray[1][0]) + abs(cornersArray[2][0] - cornersArray[3][0]))/2

            distance = 4161.416*(markWidthPix)**(-1.055)    # Marker dstance from camera, centimeters; power interpolation
            left2Right = markWidthCM * center2CenterPix / markWidthPix  # Left to right distance from center, cm
            theta = 90 - ((math.atan2(distance, left2Right)) * 180 / math.pi)     # Angle from center, degrees

            #print("Distance: ", round(distance, 2))

            if distance < stopDist:
                state = 3
                send2Duino = 3

        # Go back to search state if we can't see the id anymore
        if abs(theta > 18):
            if theta < 0:
                state = 0.5
                send2Duino = 6
            if theta > 0:
                state = 0.5
                send2Duino = 7

        if reply == 8:
            state = 0
           
    elif state == 3:
        #if reply == 0:
            #state = -1
        if reply == 1:
            state = 4
            send2Duino = 4
        if reply == 8:
            state = 0
            
    elif state == 4:
        if reply == 3:
            state = 5
            send2Duino = 5
        if reply == 8:
            state = 0
            
    elif state == 5:
        if reply == 8:
            state = 0

                
    # Loop to see if state has changed or not
    if (state != stateLast):
        print("State: ", state)
        q.put(state)
        change = True
        stateLast = state

    else:
        change = False
        stateLast = state

        loopCount += 1
        if (loopCount > 10):
            #sleep(0.1)
            loopCount = 0
            try:
                reply = i2c.read_byte_data(ARD_ADDR, int(state))
                print("Received from Arduino: ", reply)
            except IOError:
                print("Could not read from Arduino")

    
    # Show the overlay
    cv.imshow("overlay", overlay)

    # press q key to exit out of the program
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        break

# Once out of while, release the capture and destroy windows
capture.release()
cv.destroyAllWindows()