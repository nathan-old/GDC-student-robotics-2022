import time, serial
from sr.robot3 import *
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for port, desc, hwid in sorted(ports):
    SER = hwid.split(' ')[2][4:]
    print(desc)
    print((port, hwid.split(' ')[2][4:]))
    if SER == '7523031383335161C151' or '9503830363135160A1C2':
        serial_port = port
        RUGGEDUINO_ID = SER
        print(SER)
R = Robot(auto_start=True, ignored_ruggeduinos=[RUGGEDUINO_ID])

ruggeduino_device = R.ignored_ruggeduinos[RUGGEDUINO_ID]

try:
    SerialPrint = serial.Serial(serial_port)
except:
    print('Could not find Arduino')

_print = print
def print(x):
    try:
        SerialPrint.write(bytes(str(x), 'utf-8')) # send serial to arduino
        _print('-- ' + str(x))# connected
    except:
        _print('-! ' + str(x))# for not connected



R.wait_start()
num = 1
for i in range(16):
    num *= 10
    print(num)
    time.sleep(1)





'''
We will have to follow the code at:
https://studentrobotics.org/docs/programming/sr/ruggeduinos/custom_firmware#:~:text=import%20serial%0Afrom%20sr.robot3%20import%20*%0A%0ARUGGEDUINO_ID%20%3D%20%22752303138333517171B1%22%0A%0AR%20%3D%20Robot(ignored_ruggeduinos%3D%5BRUGGEDUINO_ID%5D)%0A%0Aser%20%3D%20serial.Serial(R.ignored_ruggeduinos%5BRUGGEDUINO_ID%5D)
The link should show a highlighted section in a compatable browser such a chrome.

we have to initialise the serial connection "ser = serial.Serial(port = "COM3", baudrate=9600)" onece as it will not work otherwise.




import serial
from sr.robot3 import *

RUGGEDUINO_ID = "752303138333517171B1" # whats the ID of our arduino???

R = Robot(ignored_ruggeduinos=[RUGGEDUINO_ID])

ser = serial.Serial(R.ignored_ruggeduinos[RUGGEDUINO_ID])

'''
