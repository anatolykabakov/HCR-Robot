
from lib import Robot, Serial, Planning, Tracking

ax = [0.0, 2, 2, 0]
ay = [0.0, 0, 2, 2]

if __name__ == '__main__':
    #----Init state--
    #serial   = Serial('com4', 9600)
    robot    = Robot()
    planning = Planning()
    tracking = Tracking(robot, planning, mode='pc')
    #----Planning----
    cx, cy, cyaw = planning.getTrack(ax, ay)
    #----Tracking----
    x, y         = tracking.motioncontrol(cx, cy, cyaw, target_speed=1)





 
