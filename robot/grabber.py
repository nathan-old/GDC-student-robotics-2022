import serial, serial.tools.list_ports, time



class Arduino():
    def __inint__(self):
        self.serial_num = '7523031383335161C151'
        self.RUGGEDUINO_ID = None
        self.ser = None
        print('[INITI] Arduino setup initialised')
    def Setup(self):
        '''The tools to initialise the Ruggeduino for comunications'''
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
            SER = hwid.split(' ')[2][4:]
            if SER == '7523031383335161C151':
                self.RUGGEDUINO_ID = (SER)
        return self.RUGGEDUINO_ID
    def Start(self, R, ID):
        '''Start Communications with the arduino
        R: robot class
        ID: arduino ID number
        '''
        ruggeduino_device = R.ignored_ruggeduinos[ID]
        if ruggeduino_device != None:
            self.ser = serial.Serial(ruggeduino_device, baudrate=9600)
        return self.ser


class Communicate():
    def __init__(self, serial, grab_sleep_time=3):
        print('[INITI] Arduino communications initialised')
        self.ser = serial
        self.grab_sleep_time = grab_sleep_time
    def Write(self, String):
        '''Write to the serial port of the setup arduino
        String: The string data you wish to send'''
        self.ser.write(String.encode("utf-8"))
    def Read(self):
        '''Read from the serial port of the setup arduino'''
        return self.ser.readline()
    def Grab(self):
        '''Sends the grab command to the arduino'''
        self.ser.write(b'A')
        time.sleep(self.grab_sleep_time)
    def Grab_No_Sleep(self):
        '''Sends the grab command to the arduino'''
        self.ser.write(b'A')