#import matplotlib.pyplot as plt
import time
import serial
import math
import numpy as np
import bisect
from datetime import datetime
from datetime import timedelta
from protocol import openconnect, send_msg, recieve, process_data
from ekf_slam import observation, ekf_slam

start_time = datetime.now()
L = 0.275
k = 0.5  # control gain

Kp = 1.0  # speed propotional gain
import math
import numpy as np



# EKF state covariance
Cx = np.diag([1, 1, math.radians(30.0)])**2

#  Simulation parameter
Qsim = np.diag([0.2, math.radians(1.0)])**2
Rsim = np.diag([1.0, math.radians(10.0)])**2

DT = 0.1  # time tick [s]

MAX_RANGE = 2  # maximum observation range
M_DIST_TH = 1  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM srate size [x,y]

#----------------------------------

#robot parameters
radius = 0.0682
length = L

delta = 0.1

file = open("log.txt","w")  


class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw

class Serial(object):
    def __init__(self, port, speed, timeinterval):
        """Instantiate the object."""
        super(Serial, self).__init__()
        self.connect = self.open_connect(port, speed)
        
    def open_connect(self, port, speed):
        connect = openconnect(port, speed)
        return connect
        
    def getSerialData(self):
        data = recieve(self.connect)
        self.velocity, self.yaw, self.x, self.y = process_data(data)
        return self.velocity, self.yaw, self.x, self.y

    def setSerialData(self, velocity, omega):
        send_msg(self.connect, velocity, omega)
        
class EKF_SLAM(object):
    def __init__(self):
        """Instantiate the object."""
        super(EKF_SLAM, self).__init__()
        self.xEst = np.matrix(np.zeros((STATE_SIZE, 1)))
        self.xTrue = np.matrix(np.zeros((STATE_SIZE, 1)))
        self.PEst = np.eye(STATE_SIZE)
        self.xDR = np.matrix(np.zeros((STATE_SIZE, 1)))  # Dead reckoning
        # history
        self.hxEst = self.xEst
        self.hxTrue = self.xTrue
        self.hxDR = self.xTrue

    def calc_n_LM(self, x):
        n = int((len(x) - STATE_SIZE) / LM_SIZE)
        return n


class Planning(object):
    def __init__(self, robot, serial, lidar, mode='robot'):
        """Instantiate the object."""
        super(Planning, self).__init__()
        #--------Инициализация --------
        self.ekf_slam = EKF_SLAM()
        #------------------------------
        self.robot    = robot
        self.serial = serial
        self.lidar = lidar
        self.tracking    = tracking = Tracking(robot, self, serial=serial, mode=mode, dt = 0.1)
        self.x = []
        self.y = []
        
    def getTrack(self, ax, ay):
        cx, cy, cyaw, ck, s = self.calc_spline_course(ax, ay, ds=0.1)
        return cx, cy, cyaw

    def trajectory_generator(self, ax, ay):
        cx, cy, cyaw = self.getTrack(ax, ay)
        self.trajectory_control(cx, cy, cyaw)

    def trajectory_control(self, cx, cy, cyaw):
        last_idx = len(cx) - 1# номер последней точки (массива) пути
        target_idx, _ = self.calc_target_index(self.robot, cx, cy)# номер ближайшей точки
        while last_idx > target_idx:
            target_idx, _ = self.calc_target_index(self.robot, cx, cy)# номер ближайшей точки
            x_est, y_est, yaw_est = self.state_update(cx[target_idx], cy[target_idx], cyaw[target_idx])
            self.x.append(x_est)
            self.y.append(y_est)
           # plotxy(self.x,self.y, cx, cy)
            print(millis())
        self.robot.move(self.serial, 0, 0)
        file.close()
        print('Done!')
        

    def state_update(self, cx, cy, cyaw):

        #------------------------------------------------------
        #--------------------ekf_slam--------------------------
        #------------------------------------------------------
        v, w, robotx, roboty = self.serial.getSerialData()# получаем из Ардуино: скорость, угол в рад, коорд-ы
        w = normalize_angle(w)
        self.robot.set(v, w, robotx, roboty )
        x = self.robot.x
        y = self.robot.y
        yaw = self.robot.yaw
        #------------------------------------------------------
        timestamp = millis()
        
        scan = get_scan(self.lidar, 1)
        file.write(str(str(round(timestamp, 0)) + ';' + str(x) + ';' + str(y) + ';' + str(yaw) + ';' + str(scan)) + '\n') 
        #------------------------------------------------------
##        u = np.matrix([self.robot.v, self.robot.omega]).T
##        RFID = get_LidarData()
##        self.ekf_slam.xTrue, z, self.ekf_slam.xDR, ud = observation(self.ekf_slam.xTrue, self.ekf_slam.xDR, u, RFID)
##        self.ekf_slam.xEst, self.ekf_slam.PEst = ekf_slam(self.ekf_slam.xEst, self.ekf_slam.PEst, ud, z)
##        x_state = self.ekf_slam.xEst[0:STATE_SIZE]
##        x = x_state.tolist()[0][0]
##        y = x_state.tolist()[1][0]
##        yaw = x_state.tolist()[2][0]
        #------------------------------------------------------
##        self.ekf_slam.hxEst = np.hstack((self.ekf_slam.hxEst, x_state))
##        self.ekf_slam.hxDR = np.hstack((self.ekf_slam.hxDR, self.ekf_slam.xDR))
##        self.ekf_slam.hxTrue = np.hstack((self.ekf_slam.hxTrue, self.ekf_slam.xTrue))
        #------------------------------------------------------
        #--------------------motion_control--------------------
        #------------------------------------------------------
        self.tracking.motion_control(cx, cy, cyaw, x, y, yaw, target_speed=0.2)
        #------------------------------------------------------
        #--------------------show_animation--------------------
        #------------------------------------------------------
##        show_animation = True
##        if show_animation:
##            plt.cla()
##            for i in range(len(z[:, 0])):
##                plt.plot([self.ekf_slam.xTrue[0, 0], self.ekf_slam.xEst[STATE_SIZE + i * 2]], [self.ekf_slam.xTrue[1, 0], self.ekf_slam.xEst[STATE_SIZE + i * 2 + 1]], "-k")
##
##            plt.plot(RFID[:, 0], RFID[:, 1], "*y")
##            plt.plot(self.ekf_slam.xEst[0], self.ekf_slam.xEst[1], ".r")
##
##            # plot landmark
##            for i in range(self.ekf_slam.calc_n_LM(self.ekf_slam.xEst)):
##                plt.plot(self.ekf_slam.xEst[STATE_SIZE + i * 2],
##                         self.ekf_slam.xEst[STATE_SIZE + i * 2 + 1], "xr")
##            plt.plot(cx, cy, ".r", label="course")
##            plt.plot(np.array(self.ekf_slam.hxTrue[0, :]).flatten(),
##                     np.array(self.ekf_slam.hxTrue[1, :]).flatten(), "-b")
##            plt.plot(np.array(self.ekf_slam.hxDR[0, :]).flatten(),
##                     np.array(self.ekf_slam.hxDR[1, :]).flatten(), "-g")
##            plt.plot(np.array(self.ekf_slam.hxEst[0, :]).flatten(),
##                     np.array(self.ekf_slam.hxEst[1, :]).flatten(), "-r")
##            plt.axis("equal")
##            plt.grid(True)
##            plt.pause(0.001)
        #------------------------------------------------------
        return x, y, yaw
    
    def calc_target_index(self, robot, cx, cy):
        """
        Compute index in the trajectory list of the target.

        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        """
        # Calc front axle position
        fx = robot.x
        fy = robot.y
        # Search nearest point index
        dx = [fx - icx for icx in cx]
        dy = [fy - icy for icy in cy]
        d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        error_front_axle = min(d)
        target_idx = d.index(error_front_axle)
        target_yaw = normalize_angle(np.arctan2(fy - cy[target_idx], fx - cx[target_idx]) - robot.yaw)
        if target_yaw > 0.0:
            error_front_axle = - error_front_axle
        return target_idx, error_front_axle
    
    def calc_spline_course(self, x, y, ds=0.1):
##            '''
##        интерполирует заданную траекторию кубическим сплайном
##        вход:
##        x - массив координат по x
##        y - массив координат по y
##        выход:
##        rx - массив референсных координат по x в метрах
##        ry - массив референсных координат по y в метрах
##        ryaw - массив референсных направления в рад
##        '''
        sp = Spline2D(x, y)
        s = list(np.arange(0, sp.s[-1], ds))
        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            rk.append(sp.calc_curvature(i_s))
        return rx, ry, ryaw, rk, s

class Tracking(object):
    def __init__(self, robot, planning, serial, mode, dt = None):
        """Instantiate the object."""
        super(Tracking, self).__init__()
        self.robot    = robot
        self.planning = planning
        self.serial = serial
        self.mode = mode
        self.dt = dt
        
    def pid_control(self, target, current):
##        '''
##        контроль скорости
##
##        П-регулятор
##         
##        """
        Kp = 1.5
        return Kp * (target - current)
        
    def steer_control(self, cx, cy, cyaw, x, y, yaw):
##        '''
##        функция реализует контроль ориентации робота
##        алгоритм:
##        '''
        delta = normalize_angle(cyaw - yaw)
        
        omega = delta
        if omega >1:
            omega = 1
        return omega
        
    def motion_control(self, cx, cy, cyaw, x, y, yaw, target_speed):
##        '''
##        функция реализует контроль движения по заданной траектории
##        включает в себя :
##        '''
        ai      = self.pid_control(target_speed, self.robot.v)
        omega   = self.steer_control(cx, cy, cyaw, x, y, yaw)# выход: угловая скорость поворота, и номер следующей точки
        if self.mode == 'pc':
            self.robot.update(ai, omega)
        if self.mode == 'robot':
            self.robot.move(self.serial, target_speed, omega)


class Robot(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0, dt=None):
        """Instantiate the object."""
        super(Robot, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.omega = omega
        self.dt = dt
        
    def set(self,v, yaw, x, y):
##        '''
##        вход:
##        x - положение в метрах
##        y - положение в метрах
##        yaw - курс в рад
##        v - скорость в метрах в секунду
##
##        действие:
##        устанавливает текушие скорость угол и положение 
##        '''
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, omega, dt=None):
##        '''
##        обновление состояния робота по модели одноколесного велосипеда
##        acceleration - ускорение
##        omega - угловая скорость
##        dt=None - промежуток времени
##        '''
        self.v = acceleration#*self.dt
        self.omega = omega
        self.yaw += self.omega*self.dt
        self.yaw = normalize_angle(self.yaw)
        self.x += self.v * np.cos(self.yaw) * self.dt
        self.y += self.v * np.sin(self.yaw) * self.dt

    def getPos(self):
        return self.x, self.y
      
    def move(self, obj, velocity, omega):
##        '''
##        посылает угловую и линейную скорость в порт
##        '''
        obj.setSerialData(velocity, omega)

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def millis():
   dt = datetime.now() - start_time
   ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
   return ms

def get_LidarData():
    RFID = np.array([[2.5, 0.0],
                     [0.5, 1.0],
                     [2.0, 2.0]])
    return RFID

def read_scan(ser):
    ser.write('s'.encode())
    line = ser.readline()
    s = line.decode().replace('\r\n','')
    new_s = s.split(':')
    distance_mm = float(new_s[0])
    angle_grad = float(new_s[1])
    return distance_mm, angle_grad

def get_scan(lidar_serial, coll):
    scan = []
    while len(scan) < coll:
        distance_mm, angle_grad = read_scan(lidar_serial)
        scan.append((distance_mm, angle_grad))
        print(len(scan))
    scan = points2distVec(scan)
    #print("2")
    return scan

def points2distVec(points):
    distVec = [0 for i in range(len(points))]

    for point in points: # create breezySLAM-compatible data from raw scan data
      dist = point[0]
      index = int(point[1])
      if not 0 <= index < len(points): continue
      distVec[index] = int(dist)

    # note that breezySLAM switches the x- and y- axes (their x is forward, 0deg; y is right, +90deg)
    distVec = [distVec[i-180] for i in range(0, 360)] # rotate scan data so middle of vector is straight ahead, 0deg
    return distVec

def plotKalman(hxTrue, hxK, hz, hz1):
    plt.cla()
    plt.plot(hz[:, 0], hz[:, 1], ".g")
    plt.plot(hz[:, 0], hz1[:, 1], ".b")
    plt.plot(np.array(hxTrue[0, :]).flatten(),
                np.array(hxTrue[1, :]).flatten(), c='black', label='True path')
    plt.plot(np.array(hxK[0, :]).flatten(),
               np.array(hxK[1, :]).flatten(), "-r", label='Estimated path')
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.001)
def plotxy(x,y, cx, cy):
    plt.cla()
    plt.plot(cx, cy, ".r", label="course")
    plt.plot(x, y, "b", label="course")
##    plt.plot(x_est, y_est, "-r", label="course")
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.001)

    
