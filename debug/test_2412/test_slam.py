#!/usr/bin/env python3

'''
rpslam.py : BreezySLAM Python with SLAMTECH RP A1 Lidar
                 
Copyright (C) 2018 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10



# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 200
# Laser constants (shared with Arduino)
DIST_MIN = 100; # minimum distance
DIST_MAX = 6000; # maximum distance
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
from pltslamshow import SlamShow
from components import DaguRover5, RPLIDAR
from slambotgui.slams import Slam
import matplotlib.pyplot as plt 
import math
import serial
import time		


def preprocc(ser):
    line = None
    new_x =0
    new_y =0
    old_x =0
    old_y =0
    ser.write('s'.encode())
    time.sleep(0.001)
    if(ser.inWaiting()):
        line = ser.readline()
        s = line.decode().replace('\r\n','')
        new_s = s.split(':')
        distance_mm = float(new_s[0])
        angle_grad = float(new_s[1])
        if 0 <= angle_grad <= 360 and 0 <= distance_mm <= 6000:
##            dist_meters = distance_mm/1000
##            angle_rad = angle_grad*3.14/180
##            x = dist_meters*math.cos(angle_rad)
##            y = dist_meters*math.sin(angle_rad)
            old_x = new_x
            old_y = new_y
            new_x = distance_mm
            new_y = angle_grad
            if x == "None" and y == "None": ser.write("f".encode())
            return new_x, new_y
        else: return old_x, old_y
    else :
        print("None")
        preprocc(ser)
        ser.write("f".encode())
        return old_x, old_y
    
def get_scan(lidar_serial, coll):
    scan = []
    while len(scan) < coll:
        #distance_mm, angle_grad = read_scan(lidar_serial)
        x, y = preprocc(lidar_serial)
        scan.append((x, y))
    print(len(scan))
    return scan

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    # physical objects
    robot = DaguRover5()
    laser = RPLIDAR(DIST_MIN, DIST_MAX)
    slam = Slam(robot, laser, **KWARGS) # do slam processing

    # Set up a SLAM display
    display = SlamShow(MAP_SIZE_PIXELS, MAP_SIZE_METERS*1000/MAP_SIZE_PIXELS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    #mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar
    #iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    # First scan is crap, so ignore it
    #next(iterator)

    while True:
        points = get_scan(ser, 200)
        x, y, theta, map_br = slam.updateSlam(points)

        
        # Get current robot position
##        x, y, theta = slam.getpos()
##
##        # Get current map bytes as grayscale
##        slam.getmap(mapbytes)

        # Display the map
        display.displayMap(map_br)

        # Display the robot's pose in the map
        display.setPose(x, y, theta)

        # Break on window close
        if not display.refresh():
            break


