import time, math, serial, numpy
from sr.robot3 import *


import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    SER = hwid.split(' ')[2][4:]
    print((port, hwid.split(' ')[2][4:]))
    if SER == '7523031383335161C151':
        serial_port = port
        RUGGEDUINO_ID = SER
ser = serial.Serial(serial_port)
R = Robot(auto_start=True, ignored_ruggeduinos=[RUGGEDUINO_ID])

ruggeduino_device = R.ignored_ruggeduinos[RUGGEDUINO_ID]
ser = serial.Serial(ruggeduino_device, baudrate=9600)
x_value = 0
y_value = 0

x_max = 1928
x_min = 1088
y_max = 1976
y_min = 1088
r_max = 1940
r_min = 1104
t_max = 1968
t_min = 1096

def range_map(value, max_, min_, ranges):
    return round((ranges[0] + (ranges[1] - ranges[0]) * ((value-min_)/(max_-min_))),2)

x = numpy.array([[-0.33, 0.58, 0.33], [-0.33, -0.58, 0.33], [0.67, 0, 0.33]])

R.wait_start()

while True:
##    print(i)
    arr = ser.readline().decode().rstrip().split(',')

    x_value, y_value, r_value, s_value, b_value, t_value, shutdown = int(arr[0]), int(arr[1]), int(arr[2]),int(arr[3]),int(arr[4]),int(arr[5]),int(arr[6])
    if shutdown == 0:
        shutdown_state = True
    else:
        shutdown_state = False
    if s_value < 1500:
        s_state = True
    else:
        s_state = False
    if b_value > 1500:
        b_state = True
    else:
        b_state = False
##    print(s_state, b_state, t_value)
    x_value = range_map(x_value, x_max, x_min,[-1,1])
    y_value = range_map(y_value, y_max, y_min,[-1,1])
    r_value = range_map(r_value, r_max, r_min,[-1,1])
    t_value = range_map(t_value, t_max, t_min,[0,1])
    if s_state == True:
        t_value *= -1

    y = numpy.array([[x_value], [t_value], [r_value]])

    A_speed = (round(numpy.dot(x,y)[0][0],1)) 
    B_speed = (round(numpy.dot(x,y)[1][0],1)) 
    C_speed = (round(numpy.dot(x,y)[2][0],1))
    
    if A_speed > 1:
        A_Speed = 1
    elif A_speed < -1:
        A_speed = -1
    if B_speed > 1:
        B_Speed = 1
    elif B_speed < -1:
        B_speed = -1
    if C_speed > 1:
        C_Speed = 1
    elif C_speed < -1:
        C_speed = -1
    
    R.motor_boards["SR0PJ1W"].motors[0].power = A_speed
    R.motor_boards["SR0PJ1W"].motors[1].power = B_speed
    R.motor_boards["SR0VG1M"].motors[0].power = C_speed
##    print(A_speed, B_speed, C_speed)
ser.close()





'''
http://thetechnicgear.com/2014/04/howto-build-3-wheels-holonomic-robot-using-lego
Arduino ID needs to be found and entered - Don't know how it didn't work last time I tried.
'''
