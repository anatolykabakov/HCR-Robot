


from lib import Robot, Serial, Planning, Tracking, EKF_SLAM
#lidar_port = '/dev/ttyUSB0'
import serial
lidar_serial = serial.Serial(lidar_port, 115200)
ax = [0.0, 1, 1, 2, 2]
ay = [0.0, 0, 1, 1, 0]
dt = 0.0
if __name__ == '__main__':
    #----Init state--
    #serial   = Serial('/dev/ttyACM0', 115200, timeinterval=dt)
    robot    = Robot(x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0, dt=dt)
    planning = Planning(robot, serial, lidar_serial,  mode='pc')
    planning.trajectory_generator(ax, ay)























 

