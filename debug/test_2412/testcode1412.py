


from lib import Robot, Serial, Planning, Tracking, EKF_SLAM

ax = [0.0, 1, 1, 2, 2]
ay = [0.0, 0, 1, 1, 0]
dt = 0.0
if __name__ == '__main__':
    #----Init state--
    serial   = Serial('com3', 115200, timeinterval=dt)
    robot    = Robot(x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0, dt=dt)
    planning = Planning(robot, serial, mode='robot')
    planning.trajectory_generator(ax, ay)























 

