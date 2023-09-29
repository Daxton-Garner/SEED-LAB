# Jordie Weber
# SEED Lab - Computer Vission Assignment 2a

# A lot of imports
import time
import board
import numpy as np
import cv2 as cv
from cv2 import aruco
from time import sleep
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# imports for Arduino communications
from smbus2 import SMBus
ARD_ADDR = 8
i2c = board.I2C()
i2c = SMBus(1)

# lcd initialization, set columns/rows, color, text direction
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [50, 0, 50]
lcd.text_direction = lcd.LEFT_TO_RIGHT
outputString = " "
previousOutput = "."
sendToDuino = "0"

# Disctionary for aruco generation and detection
myDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

# This code generates markers, uncomment to display a marker
#myAruco = aruco.GridBoard((1,1),4.0,1.0,myDict)
#myArucoImg = myAruco.generateImage((500,500))
#cv.imshow("myArucoImg",myArucoImg)
#cv.imwrite("Aruco.jpg", myArucoImg)

# Capture from the camera plugged into the Pi
capture = cv.VideoCapture(0)

# Set dimensions of the capture and find the center of the window
capWidth = 640
capHeight = 480
xCapCent = capWidth/2
yCapCent = capHeight / 2
capture.set(cv.CAP_PROP_FRAME_WIDTH, capWidth)


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

        # Calculate where the center of the marker is on the screen
        cornersArray = np.array(corners)[0][0]
        xMarkCent = (cornersArray[0][0] + cornersArray[1][0] + cornersArray[2][0] + cornersArray[3][0]) / 4 
        yMarkCent = (cornersArray[0][1] + cornersArray[1][1] + cornersArray[2][1] + cornersArray[3][1]) / 4
        # Compare center of marker to center of screen to locate marker in capture
        if (xMarkCent < xCapCent) and (yMarkCent < yCapCent):
            outputString = "NW"
            sendToDuino = "1"
            # use I2C to communicate with Arduino
            i2c.write_byte_data(ARD_ADDR, sendToDuino, outputString)
            sleep(0.1)
            reply = i2c.read_byte_data(ARD_ADDR, sendToDuino)
            lcd.message = "Marker in:\n NW quadrant!"

        elif (xMarkCent < xCapCent) and (yMarkCent > yCapCent):
            outputString = "SW"
            sendToDuino = "4"
            # use I2C to communicate with Arduino
            i2c.write_byte_data(ARD_ADDR, sendToDuino, outputString)
            sleep(0.1)
            reply = i2c.read_byte_data(ARD_ADDR, sendToDuino)
            lcd.message = "Marker in:\n SW quadrant!"

        elif (xMarkCent > xCapCent) and (yMarkCent < yCapCent):
            outputString = "NE"
            sendToDuino = "2"
             # use I2C to communicate with Arduino
            i2c.write_byte_data(ARD_ADDR, sendToDuino, outputString)
            sleep(0.1)
            reply = i2c.read_byte_data(ARD_ADDR, sendToDuino)
            lcd.message = "Marker in:\n NE quadrant!"
            
        else:
            outputString = "SE"
            sendToDuino = "3"
            # use I2C to communicate with Arduino
            i2c.write_byte_data(ARD_ADDR, sendToDuino, outputString)
            sleep(0.1)
            reply = i2c.read_byte_data(ARD_ADDR, sendToDuino)
            lcd.message = "Marker in:\n SE quadrant!"

        # Edit the overlay display to outline the marker
        ids = ids.flatten()
        for (outline, id) in zip(corners, ids):
            markerCorners = outline.reshape((4,2))
            overlay = cv.putText(overlay, str(id), (int(markerCorners[0,0]),int(markerCorners[0,1])-15),cv.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)
    else:
        # If a marker is not found
        outputString = "No markers found"
        sendToDuino = "0"
        lcd.message = "No Marker Found!"

    
    # Show the overlay
    cv.imshow("overlay", overlay)

    # Write the output string to the lcd screen, keep track of chnages in output
    if outputString != previousOutput: lcd.clear()
    lcd.message = outputString
    previousOutput = outputString

    # press q key to exit out of the program
    k = cv.waitKey(1) & 0xFF
    if k == ord('q'):
        break

# Once out of while, release the capture and destroy windows
capture.release()
cv.destroyAllWindows()

                                 
