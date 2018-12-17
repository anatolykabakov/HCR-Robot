
import matplotlib.pyplot as plt 
import math
import serial



def preprocc(ser):
    ser.write('s'.encode())
    line = ser.readline()
    s = line.decode().replace('\r\n','')
    new_s = s.split(':')
    distance_mm = float(new_s[0])
    angle_grad = float(new_s[1])
    dist_meters = distance_mm/1000
    angle_rad = angle_grad*3.14/180
    x = dist_meters*math.cos(angle_rad)
    y = dist_meters*math.sin(angle_rad)
    return x, y


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB2', 115200)
    x_mass = []
    y_mass = []
    while True:
        x, y = preprocc(ser)
        x_mass.append(x)
        y_mass.append(y)
        plt.plot(y_mass, x_mass, ".b")
        plt.pause(0.001)
