import serial
try:
    ser = serial.Serial(port = "COM3")#, baudrate=9600)# will need to change COM3 to the right address
except:
    print('Could not find Arduino')
_print = print
def print(x):
    try:
        ser.write(bytes(str(x), 'utf-8')) # send serial to arduino
        _print('-- ' + str(x))# connected
    except:
        _print('-! ' + str(x))# for not connected
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
