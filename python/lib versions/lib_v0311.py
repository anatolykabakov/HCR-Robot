import matplotlib.pyplot as plt
import time
import serial
import math
import numpy as np
import bisect
from datetime import datetime
from datetime import timedelta
from protocol import openconnect, send_msg, recieve, process_data

start_time = datetime.now()
L = 0.275
k = 0.5  # control gain

Kp = 1.0  # speed propotional gain
import math
import numpy as np
#Sensors parameters
R1 = np.diag([1, 1])**2#степень доверия 1 датчика
R2 = np.diag([1, 1])**2#степень доверия 2 датчика

#  Simulation parameter
Qsim = np.diag([0.3, 0.3])**2
Rsim = np.diag([5.0, math.radians(30.0)])**2
Qsim1 = np.diag([0.9, 0.9])**2
Rsim1 = np.diag([5.0, math.radians(30.0)])**2

#robot parameters
radius = 0.0682
length = L

delta = 0.1
class Model():
    def __init__(self, radius, length):
        #robot parameters
        self.radius = radius
        self.length = length
        
        self.delta = 0.1

        self.model  = np.matrix(np.zeros((4, 1)))
        self.hxModel = np.matrix(np.zeros((4, 1)))
        
        
        
    def motion_model(self, x, u):
        """
        Unicycle Motion Model

        motion model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}

        """

        A = np.matrix([[1.0, 0, 0, 0],
                   [0, 1.0, 0, 0],
                   [0, 0, 1.0, 0],
                   [0, 0,   0, 0]])

        B = np.matrix([[delta * math.cos(x[2, 0]), 0],
                   [delta * math.sin(x[2, 0]), 0],
                   [0.0, delta],
                   [1.0, 0.0]])

        x = A * x + B * u

        return x

    def get_motion_model(self, u):
        self.model  = self.motion_model(self.model, u)
        self.hxModel = np.hstack((self.hxModel, self.model))
        return self.model, self.hxModel
    
               
    
    def diff_motion_model(self, model, v):
        """
        Differential Motion Model

        motion model
        x_{t+1} = x_t+radius*(Vl + Vr)*0.5*dt*cos(yaw)
        y_{t+1} = y_t+radius*(Vl + Vr)*0.5**dt*sin(yaw)
        yaw_{t+1} = yaw_t+radius/length*(Vl - Vr)*dt

        """
        F = np.matrix([[1.0, 0, 0, 0],
                   [0, 1.0, 0, 0],
                   [0, 0, 1.0, 0],
                   [0, 0,   0, 1.0]])

        B = np.matrix([[delta * radius * 0.5 * (v[0, 0] + v[1, 0]) * math.cos(model[2, 0])],
                   [delta * radius * 0.5 * (v[0, 0] + v[1, 0]) * math.sin(model[2, 0])],
                   [delta * (radius / length)*(v[1, 0] - v[0, 0])],
                   [1.0]])

        model = F * model + B 

        return model



    def uni_to_diff_model(self, u):
        '''
        v_r = ((2 * v) + (w * L)) / (2 * R)
        v_l = ((2 * v) - (w * L)) / (2 * R)
        '''
    
        Vr = ((2*u[0, 0]) + (u[1, 0]*length))/ (2*radius)
        Vl = ((2*u[0, 0]) - (u[1, 0]*length))/ (2*radius)
        V = np.matrix([Vl, Vr]).T
        return V

    def diff_to_uni(self, v):
        '''    
        v = ( R / 2.0 ) * ( v_r + v_l )
        omega = ( R / L ) * ( v_r - v_l )
        '''
        vel = (radius/2) * (v[0, 0] + v[1, 0])
        w = (radius/length) * (v[1, 0] - v[0, 0])

        u = np.matrix([vel, w]).T#[v, omega]
    
        return u
    
class EKF():
    def __init__(self):
        # Model parameters
        self.Q = np.diag([0.1, 0.1, math.radians(1.0), 1.0])**2#степень доверия модели движения 
        # Initial state

        #Predict
        self.xPred = np.matrix(np.zeros((4, 1)))
        #Update
        self.DT = 0.1
        self.xEst = np.matrix(np.zeros((4, 1)))
        self.PEst = np.eye(4)
        

    
    def jacobF(self, x, u):

        yaw = x[2, 0]
        v = u[0, 0]
        jF = np.matrix([
        [1.0, 0.0, -self.DT * v * math.sin(yaw), self.DT * math.cos(yaw)],
        [0.0, 1.0, self.DT * v * math.cos(yaw), self.DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

        return jF
    def jacobH(self, x):
    # Jacobian of Observation Model
        jH = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

        return jH
    
    def observation_model(self, x):
        
        #  Observation Model
        H = np.matrix([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

        z = H * x
        return z

    def predict(self, model, u):
        #Predict
        self.xPred = model#среднее значение модели 
        jF = self.jacobF(self.xPred, u)#среднеквадратичное отклонение модели движения(ковариация)
        self.PPred = jF * self.PEst * jF.T + self.Q#Предсказание ошибки ковариации(среднеквадратичное отклонение модели + среднекрадратичное откронение предсказания(Q))


    def update(self, z, R):
        #  Update
        jH = self.jacobH(self.xPred)#функция измерения 
        zPred = self.observation_model(self.xPred)#предсказание среднего значения измерения датчика 
        y = z.T - zPred#среднее значение реальных измерений датчика - среднее значение предстаказанного(идеального) значения измерений датчика
        S = jH * self.PPred * jH.T + R#предсказание среднеквадратичного отклонения для измерений датчика
        K = self.PPred * jH.T * np.linalg.inv(S)#Вычисляем усиление Калмана, с помощью подсчета ошибки вычислений
        self.xEst = self.xPred + K * y#оптимизированная модель, за счет усиления среднего значения измерений датчика
        self.PEst = (np.eye(len(self.xEst)) - K * jH) * self.PPred#Обновление ошибки ковариации модели движения 



    def get_state(self):
        return self.xEst, self.PEst

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
        #self.connect = serial.Serial(port, speed)
        self.timeinterval = timeinterval
        self.velocity = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.connect = self.open_connect(port, speed)
        

    def open_connect(self, port, speed):
##        connect = serial.Serial(port, speed)
##        print("open port")
##        time.sleep(1)
        connect = openconnect(port, speed)
        return connect
    
    def getSerialData(self):
        data = recieve(self.connect)
        self.velocity, self.yaw, self.x, self.y = process_data(data)

##        data = self.connect.readline()
##        data = data.decode().replace('\r\n', '')
##        data = data.split('; ')
##        self.connect.flushInput()
##        if len(data)==5:
##            self.velocity = float(data[0])
##            self.yaw    = float(data[1])
##            self.x = float(data[2])
##            self.y = float(data[3])
##            print('receive: '+ str(data))
##            return self.velocity, self.yaw, self.x, self.y
        
        return self.velocity, self.yaw, self.x, self.y
    
##    def openconnect(port, speed):
##        connect = serial.Serial(port, speed)
##        time.sleep(1)
##        is_connected = False
##        while not is_connected:
##            print("Waiting for arduino...")
##            connect.write(start_connect.encode())
##            connect_flag = connect.read(1).decode()
##            check_connect(connect)
##            if not connect_flag:
##                time.sleep(0.1)
##                continue
##            if connect_flag == 'r':
##                is_connected = True
##        return connect
##
##    def process_data(data):
##    data = data.split(';')
##    velocity = float(data[0])
##    yaw    = float(data[1])
##    x = float(data[2])
##    y = float(data[3])
##    return velocity, yaw, x, y

    def receiving(self):
        '''
        получаем данные из порта
        '''
        buffer_string = self.connect.read(self.connect.inWaiting())
    
        lines = buffer_string.decode().split('\n')
        #print(lines)
        last_received = lines[-2]
        data = last_received.split(';')
        velocity = float(data[0])
        yaw    = float(data[1])
        x = float(data[2])
        y = float(data[3])
        buffer_string = lines[-1]

        print('receive: '+ str(data) + ' ' + str(millis()))
        return velocity, yaw, x, y

    def setSerialData(self, velocity, omega):
        #self.connect.write(data.encode())
        send_msg(self.connect, velocity, omega)
        #time.sleep(self.timeinterval)

class Planning(object):
    def __init__(self):
        """Instantiate the object."""
        super(Planning, self).__init__()
        #--------Инициализация --------
        self.kalman = EKF()
        #robot = Model(radius_wheel, length)
        self.PEst = np.eye(4)
        self.estimation = np.matrix(np.zeros((4, 1)))
        self.hz = np.zeros((1, 2))
        self.hz1 = np.zeros((1, 2))
        self.hxK = self.estimation
        self.model  = np.matrix(np.zeros((4, 1)))
        self.hxModel = np.matrix(np.zeros((4, 1)))
        #------------------------------
        
        
    def getTrack(self, ax, ay):
        cx, cy, cyaw, ck, s = self.calc_spline_course(ax, ay, ds=0.1)
        return cx, cy, cyaw

    def motion_state(self, robot):
        #------------- Картографирование с данных сенсоров ------
        #1.получить массив состоящий из x y alpha от лидара
        #2.получить x y aplha с одометра
        #3....еще сенсоры?
        #4. линеризовать массив лидара, получить гладкие линии на карте
        #5. получить текущее положение с лидара
        #------------- Локализация фильтром Калмана        ------
        #1. получить текущее положение с одометра
        model = np.matrix([robot.x, robot.y, robot.yaw, robot.v]).T
        #2.получить управляющее воздействие u = [v, yaw]
        u = np.matrix([robot.v, robot.yaw]).T
        #2. получить текущее положение с лидара
        #3. получить оптимизированное фильтром Калмана значение

##        for i in range(int(seconds/dt)):
##        model, hxModel  = robot.get_motion_model(u)
        z, z1 = get_gps_data(model)#получение данных с сенсоров для сравнения
        self.kalman.predict(model, u)#текущее состояние модели 
        self.kalman.update(z, R1)# корректировка состояния модели на основе данных с сенсора 1
        self.kalman.update(z1, R2)# корректировка состояния модели на основе данных с сенсора 2
        self.estimation, self.PEst = self.kalman.get_state()#получение оптимизированного значения
        self.hxK = np.hstack((self.hxK, self.estimation))
        self.hz = np.vstack((self.hz, z))
        self.hz1 = np.vstack((self.hz1, z1))
        self.hxModel = np.hstack((self.hxModel, model))
        #plotKalman(self.hxModel, self.hxK, self.hz, self.hz1)
        #------------- Генерация новых траекторий          ------
        #1. условие - если есть препядствие - сменить траекторию
        
    
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
    def __init__(self, robot, planning, serial, mode, dt = 0.5):
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
##        алгоритм:
##        1. сравнивает заданную скорость движения с текущей,
##        если они не равны умножаем разницу на кожффициент усиления 
##        '''
##        """
##         Proportional control for the speed.
##        """
        Kp = 0.2
        return Kp * (target - current)
        
    def steer_control(self, cx, cy, cyaw):
##        '''
##        функция реализует контроль ориентации робота
##        алгоритм:
##        1. определяем текущую референсную точку, к которой стремимся
##        2. определяем направление референсной точки
##        3. определяем разницу между направлением референскной точкой и текущим направлением робота
##        4. вычисляем разницу - угол на который нужно повернуть.
##        5. вычисляем необходимую угловую скорость, чтобы повернуть к референсному направлению за промежуток времени
##        '''
        current_target_idx, error_front_axle = self.planning.calc_target_index(self.robot, cx, cy)
        delta = normalize_angle(cyaw[current_target_idx] - self.robot.yaw)
        #print(str(cyaw[current_target_idx])+' - '+str(self.robot.yaw))
        omega = delta/0.25
        print(omega)
        return omega, current_target_idx
        
    def motioncontrol(self, cx, cy, cyaw, target_speed):
##        '''
##        функция реализует контроль движения по заданной траектории
##        включает в себя :
##        1. контроль позиционирования - определение текущего местоположения
##        2. контроль ориентации - поддержание заданного курса
##        3. отправку управляющих воздействий в порт
##        '''
        last_idx = len(cx) - 1# номер последней точки (массива) пути
        target_idx, _ = self.planning.calc_target_index(self.robot, cx, cy)# номер ближайшей точки
        x   = [self.robot.x]# массив коорд точек пройденной траектории
        y   = [self.robot.y]
        while last_idx > target_idx:
            if self.mode == 'robot':
                v, w, robotx, roboty = self.serial.getSerialData()# получаем из Ардуино: скорость, угол в рад, коорд-ы
                w = normalize_angle(w)
                self.robot.set(v, w, robotx, roboty )
            ai                = self.pid_control(target_speed, self.robot.v)
            omega, target_idx = self.steer_control(cx, cy, cyaw)# выход: угловая скорость поворота, и номер следующей точки
            if self.mode == 'pc':
                self.robot.update(target_speed, omega)
                self.planning.motion_state(self.robot)
            if self.mode == 'robot':
                v = ai*self.robot.dt
                self.robot.move(self.serial, target_speed, omega)
            x.append(self.robot.x)
            y.append(self.robot.y)
            plotxy(x,y, cx, cy)
        if self.mode == 'robot':
            self.robot.move(self.serial, 0, 0)
        return x, y

class Robot(object):


    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, omega=0, dt=0.2):
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
        self.v = acceleration*self.dt
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
        #data = str(velocity) + ' ' + str(omega)
        #print('send: '+ data + ' ' + str(millis()))
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




def f(v):
    k = 255
    return int(k*v)
def enc2odom(encoders):
    #Interval * 428 / 663 / 1000000
    nl = float(encoders[0])#имп/сек
    nr = float(encoders[1])#имп/сек
    Sl = (nl/668)*float(2*3.14*radius)
    Sr = (nr/668)*float(2*3.14*radius)
    
##    Sl = nl * 428 / 663 / 10000
##    Sr = nl * 428 / 663 / 10000

    return [Sl, Sr]
def odom2vel(odometers):
    ##    #Dr = C*Nr
    Sr = odometers[0]
    Sl = odometers[1]
    Sc = (Sr+Sl)/2
    #Sc = (Sl+Sr)/2
    Vl = Sl/dt
    Vr = Sr/dt
    Vc = Sc/dt
    #angle = (radius/length) * (v[1] - v[0])
    vel = [Vl, Vr, Vc]
    return vel

def diff_to_uni(v):
        '''    
        v = ( R / 2.0 ) * ( v_r + v_l )
        omega = ( R / L ) * ( v_r - v_l )
        '''
        omegaL=v[0]/radius#угловая скорость
        omegaR=v[1]/radius
        vel = (radius/2)*(omegaL + omegaR)#m/s
        angle = (radius/length) * (omegaR - omegaL)#rad/sec
        #angle = angle*180/3.14#градус

        #u = np.matrix([vel, w]).T#[v, omega]
        #u = [vel, angle]
        return vel, angle



def uni_to_diff_model(v, angle):
        '''
        v_r = ((2 * v) + (w * L)) / (2 * R)
        v_l = ((2 * v) - (w * L)) / (2 * R)
        '''
    
        Vr = ((2*v) + (angle*length))/ (2*radius)#rad/sec
        Vl = ((2*v) - (angle*length))/ (2*radius)
        Vr = Vr*radius
        Vl = Vl*radius#meters per sec
        #V = [Vl, Vr]
        return [Vl, Vr]
    
def read_port(ser):
    data = ser.readline()
    data = data.decode().replace('\r\n', '')
    data = data.split('; ')
    if len(data) == 5:
        return data

def getDataFromRobot(ser):
    data = read_port(ser)
    v = float(data[0])
    w = float(data[1])
    return v, w



def data2odom(data):
    wl = data[0]
    wr = data[1]
    odom = [wl, wr]
    val = enc2vel(odom)
    return odom

def read_file(path):
    odometr = []
    with open(path,'r') as f:
        for line in f:
            data = line.split('; ')
            data = [float(data[0]), float(data[1])]
            odometr.append(data)
    return odometr
    

def calc_v(odometr):
    Vel = []
    for i, item in enumerate(odometr):
        #print(i)
        Nl = odometr[i][0]
        Nr = odometr[i][1]
        Dl = C*Nl
        Dr = C*Nr
        Dc = (Dl+Dr)/2
        Vl = Dl/dt
        Vr = Dr/dt
        Vc = Dc/dt
        Vel.append([Vl, Vr, Vc])
    return Vel

def ret_pos(model):
    z = np.matrix(np.array([model[0, 0], model[1, 0]]))
    return z

def get_v():
    log = read_file('odometr.log')
    v = calc_v(log)
    return v

def get_gps_data(model):
    # add noise to gps x-y
    zx = model[0, 0] + np.random.randn() * Qsim[0, 0]
    zy = model[1, 0] + np.random.randn() * Qsim[1, 1]
    z = np.matrix([zx, zy])

    zx = model[0, 0] + np.random.randn() * Qsim1[0, 0]
    zy = model[1, 0] + np.random.randn() * Qsim1[1, 1]
    z1 = np.matrix([zx, zy])
    return z, z1

def millis():
   dt = datetime.now() - start_time
   ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
   return ms

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
    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.001)
