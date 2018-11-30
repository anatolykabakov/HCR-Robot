

import serial
import math
from time import sleep
import threading
import numpy as np 
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
p = GPIO.PWM(18, 2000)
p.start(100)

Start_Scan = "\xA5\x20" #Begins scanning
Force_Scan = "\xA5\x21" #Overrides anything preventing a scan
Health = "\xA5\x52" #Returns the state of the Lidar
Stop_Scan = "\xA5\x25" #Stops the scan
RESET = "\xA5\x40" #Resets the device

def getScan(port, col):
        #set the port as an instance variable
        
        port = port
        #lock checks if the connection is made
        lock = False 

        #Begin by starting the scan
        lock = startScan(port)

        #Once scan is started, beging printing data
        if lock == True:
            scan = getPoints(port, col)
        else:
            print "Exiting"
        return scan

def startScan(port):
        print "Connecting"
        line = ""
        #Lock is true once connected
        lock = False
        #Continue looping until connected
        while lock == False:
            #print "..."
            # First reset the port
            port.write(RESET)
            # Wait
            sleep(2)
            #Start reading
            #Look for the correct start
            #frame of A55A
            port.write(Start_Scan)
            try:
                #If after looping nothing found,
                #Reset and try again
                for a in range(0, 250):
                    character = port.read()
                    line += character
                    if (line[0:2] == "\xa5\x5a"):
                        if(len(line) == 7):
                            lock = True
                            break
                        
                    elif (line[0:2] != "\xa5\x5a" and len(line) == 2):
                        line = ""
            except KeyboardInterrupt:
                break
        return lock
    
def getPoints(port, col, polar=True):
        scan = []
        del scan[:]
        line = ""
        while len(scan)!=col:
            try:
                character = port.read()
                line += character
                #Data comes in 5 byte blocks
                if (len(line) == 5):
                    #Switches based on desired output
                    if polar == True:
                        point = str(point_Polar(line))
                    else:
                        point = str(point_XY(line))
                    #print point
                    scan.append(point)
                    line = ""
                    
            except KeyboardInterrupt:
                break
        return scan
    
def leftshiftbits(line):
        line = int(line, 16)
        line = bin(line)
        line = line[:2] + "0" + line[2:-1]
        line = int(line, 2) #convert to integer
        return line
    
def point_Polar(serial_frame,radians=False):
        #Get Distance
        distance = serial_frame[4].encode("hex") + serial_frame[3].encode("hex")
        distance = int(distance, 16)
        distance = distance / 4 #instructions from data sheet
        #Get Angle
        angle = serial_frame[2].encode("hex") + serial_frame[1].encode("hex")
        angle = leftshiftbits(angle) #remove check bit, convert to integer
        angle = angle/64 #instruction from data sheet

        if radians == True:
            theta = (angle * np.pi) / 180 #uncomment to use radians
        
            return(distance,theta) #uncomment to return radians

        else:
            return(distance, angle)
        
def point_XY(serial_frame):
        circular_coordinates = point_Polar(serial_frame)
        distance = circular_coordinates[0]
        angle = circular_coordinates[1]
        
        #Get X
        x = distance * math.cos(angle)
        
        #Get Y
        y = distance * math.sin(angle)
        return (x,y)
    
if __name__ == "__main__":
    #COM4 was used on my computer, this will change based on
    #your setup and whether you're on Windows/Mac/Linux
    port = '/dev/ttyUSB0'
    ser = serial.Serial(port, 115200, timeout = 5)
    ser.setDTR(False)
    print ser.name

    #Create a Lidar instance, this will immidiately start printing.
    #To edit where the data is sent, edit the GetPoints Method
    lidar = getScan(ser, 270)
    print len(lidar)
    for i in lidar:
        print(i)

