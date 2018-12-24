
import serial
import time
port = 'com5'
speed = 115200
set_command = 'v'
print_command = 'd'
start_connect = 's'
linearVelocity = 1
angularVelcity = 0.1
def check_connect(connect):
    c = connect.read(1).decode()
##    if c == 'c':
##        print("true")
##    else:
##        print("false read")
        
def send_msg(connect, a, b):
    send_data = set_command + str(a) + ' ' + str(b) + "\n"
    connect.write(send_data.encode())
    check_connect(connect)

def recieve(connect):
    connect.write(print_command.encode())
    recieve_data = connect.read(24).decode() # чтение строки из 24 символов в строку
    check_connect(connect)
    return recieve_data

def openconnect(port, speed):
    connect = serial.Serial(port, speed)
    time.sleep(1)
    while not connect.is_open:
        openconnect(port, speed)
    is_connected = False
    while not is_connected:
        print("Waiting for arduino...")
        connect.write(start_connect.encode())
        connect_flag = connect.read(1).decode()
        check_connect(connect)
        if not connect_flag:
            time.sleep(0.1)
            continue
        if connect_flag == 'r':
            is_connected = True
            print('Connected!')
    return connect
    
def process_data(data): # разбиваем строку на отдельные значения 
    data = data.split(';')
    velocity = float(data[0])
    yaw    = float(data[1])
    x = float(data[2])
    y = float(data[3])
    return velocity, yaw, x, y
if __name__ == '__main__':

    connect = openconnect(port, speed)
##    print('Connected!')

    send_msg(connect, linearVelocity, angularVelcity)
    data = recieve(connect)
    print(data)
    velocity, yaw, x, y = process_data(data)
    print('receive: '+ str(velocity) +' '+str(yaw)+ ' ' +str(x) +' '+str(y))
##    send_msg(connect, 2, 2)
##    data = recieve(connect)
##    print(data)
##    send_msg(connect, 3, 3)
##    data = recieve(connect)
##    print(data)
    


