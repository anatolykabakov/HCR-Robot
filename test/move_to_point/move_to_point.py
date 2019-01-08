from lib import Serial
import matplotlib.pyplot as plt
import time
import math
ERROR_POS = 0.1
class Controller(object):
    def __init__(self, v, w):
        self.v = v
        self.w = w
        self.x_ref = 0
        self.y_ref = 0
        self.pos_error = 0

    def position_controller(self, x_goal, y_goal, x, y, theta, delta_time):
        x_diff = x_goal - x
        y_diff = y_goal - y
        delta_l = math.sqrt(x_diff**2 + y_diff**2)
        e_alpha = math.atan2(y_diff, x_diff) - theta
        e_rho = delta_l*math.cos(e_alpha)
        self.pos_error = delta_l

        # 
        self.w = e_alpha#/delta_time
        self.v = e_rho
        if self.v > 0.3:
            self.v = 0.3
        if self.v < -0.30:
            self.v = -0.30
        if self.w > 1:
            self.w = 1
        if self.w < -1:
            self.w = -1
        if delta_l < ERROR_POS:
            self.w = 0
            self.v = 0        
        
if __name__ == '__main__':
    #----Init state--
    
    serial   = Serial('com4', 57600)
    controller = Controller(v=0, w=0)
    x = []
    y = []
    v = []
    w = []
    v.append(controller.v)
    w.append(controller.w)
    x_ref = 1
    y_ref = 1
    prev_time = 0
    current_time = 0
    delta_time = 0

    controller.pos_error = 1
    while abs(controller.pos_error) > 0.1:
        vel, robotyaw, robotx, roboty = serial.getSerialData()
        prev_time = current_time
        current_time = time.time()
        delta_time = current_time - prev_time
        controller.position_controller(x_ref, y_ref, robotx, roboty, robotyaw, delta_time)
        serial.setSerialData(controller.v, controller.w)
        x.append(robotx)
        y.append(roboty)
        v.append(controller.v)
        w.append(controller.w)
        plt.plot(x_ref, y_ref, ".r")
        plt.plot(x, y, "-b")
        plt.pause(0.001)


##    plt.plot(pos_error, "--r", label='ошибка позиционирования' )
    plt.subplots(1)
    plt.plot(v, "-b", label='Линейная скорость')
    plt.plot(w, "--r", label='Угловая скорость' )

    plt.show()
