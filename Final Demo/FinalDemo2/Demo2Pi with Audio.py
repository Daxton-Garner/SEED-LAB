# SEED Lab - Demo 2 - Computer Vission
# Purpose: State machine and calculations for demo 2.
# Method: This code communicates with the arduino and tells it which state the bot is in so that
#           the robot moves as desired. The pi uses computer vision to tell the arduino how far
#           away the marker is from the camera and what angle it is at in the camera.
# Required hardware: Camera, connection to Arduino.


# Initialization
import time
import board
import numpy as np
import cv2 as cv
import threading
import queue
import math
import pygame

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
stopDist  = 30.48 * 1.5     # Multiply by 1.5 for fudge factor
loopCount = 0
Flag = False
FlagLast = False
Kachow = False

# Communication variables
ARD_ADDR = 8
offset = 1
i2c = board.I2C()
i2c = SMBus(1) 


# Dictionary for aruco generation and detection
myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

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

# Threading for arduino communication     
def ArduinoComThread(): 
    while True: 

        if change:
            gotSomething = q.get()
            if(gotSomething != -9):
                try:
                    i2c.write_byte_data(ARD_ADDR, offset, int(send2Duino))  
                except IOError:
                    print("Could not write data toArduino")

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

    # Not searching yet
    if state == 0:  
        state = 0.5
        send2Duino = 1
        reply = 0
        pygame.mixer.init()
        pygame.mixer.music.load("Life is a Highway.mp3")
        pygame.mixer.music.set_volume(0.5)
        pygame.mixer.music.play()


    # Searching but marker isn't seen yet
    elif state == 0.5:  
        if not ids is None:
            state = 1
            send2Duino = 1
        if reply == 8:
            state = 0

    # Searching but marker is seen
    elif state == 1:    
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

            #print("Theta: ", round(theta, 2))
            #print(theta)

            # When the marker is within 15deg, go to next state
            if abs(theta) < 5:
                state = 2
                send2Duino = 2
                
        if reply == 8:
            state = 0

    elif state == 2:    # Marker is in the middle of the screen
        send2Duino = 2
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

            #print(theta)

            # Go back to search state if we can't see the id anymore
            if abs(theta) > 5:
                if theta < 0:
                    state = 6
                    send2Duino = 6
                if theta > 0:
                    state = 7
                    send2Duino = 7

            # When the marker is within the stop distance then go to next state
            if distance < stopDist:
                state = 3
                send2Duino = 3

            if reply == 8:
                state = 0

    # State for angle - deg
    elif state == 6:
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

            #print(theta)
            #print(send2Duino)

            if distance < stopDist:
                state = 3
                send2Duino = 3
        
            if theta > -5:
                send2Duino = 2
                state = 2
            #else:
                #state = 6
                #send2Duino= 6
                
        if reply == 8:
            state = 0

    # State for angle + deg
    elif state == 7:
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

            #print(theta)
            #print(send2Duino)

            if distance < stopDist:
                state = 3
                send2Duino = 3

            if theta < 5:
                send2Duino = 2
                state = 2
            #else:
                #state = 7
                #send2Duino= 7
            
        if reply == 8:
            state = 0


# These next states are handled by the arduino, this is just for changing states
    elif state == 3:
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
        if reply == 9:
            Flag = True
            print(reply)
        
    if Kachow == True:
        pygame.mixer.music.load("Kachow.mp3")
        pygame.mixer.music.play()
        pygame.mixer.music.set_volume(1.0)
                
    # Loop to see if state has changed or not
    if (state != stateLast):
        print("State: ", state)
        q.put(send2Duino)
        change = True
        stateLast = state
    else:
        change = False
        stateLast = state

        loopCount += 1
        if (loopCount > 10):
            loopCount = 0
            try:
                reply = i2c.read_byte_data(ARD_ADDR, int(state))
                #print("Received from Arduino: ", reply)
            except IOError:
                print("Could not read from Arduino")
                
    if (Flag != FlagLast):
        Kachow =True
        FlagLast = Flag
    else:
        Kachow = False
        FlagLast = Flag
        
    # Show the overlay
    cv.imshow("overlay", overlay)

    # press q key to exit out of the program
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        break

# Once out of while, release the capture and destroy windows
capture.release()
cv.destroyAllWindows()
