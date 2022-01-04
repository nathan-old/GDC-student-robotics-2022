from sr.robot3 import *
import time, math

R = Robot(auto_start=True, verbose=True)
R._log_discovered_boards
def motors(power = 1):
    R.motor_boards['SR0PJ1W'].motors[0].power = power
    R.motor_boards['SR0PJ1W'].motors[1].power = power
    R.motor_boards['SR0VG1M'].motors[0].power = power
    R.motor_boards['SR0VG1M'].motors[1].power = power # remove for three wheel robot
def motor(power = 1, motor = 0):
    if motor == 0:
        R.motor_boards['SR0PJ1W'].motors[0].power = power
    if motor == 1:
        R.motor_boards['SR0PJ1W'].motors[1].power = power
    if motor == 2:
        R.motor_boards['SR0VG1M'].motors[0].power = power
    if motor == 3:
        R.motor_boards['SR0VG1M'].motors[1].power = power
        
def servo(pos = 0, servo = 0):
    R.servo_boards['sr0GG39'].servos[servo].position = pos


def turn_angle(angle, radius = 0.18):# set radius of arms (in m)
##    if angle > 180:
##        angle = 180 - angle
    distance = angle*((math.pi*2*radius)/360)
    time_, power = move(distance)
    motors(power)
    time.sleep(time_)
    motors(BRAKE)

def move(distance, rpm = 140, circumference = 0.15):# set rpm at power 1 and -1 and circumference of wheels(m)
    power = 1 
    if distance < 0:
        distance *= -1
        power = -1
    rps = rpm/60
    speed = circumference*rps
    time_ = float(distance/speed)
    return time_, power

tones = {
"B0": 31,
"C1": 33,
"CS1": 35,
"D1": 37,
"DS1": 39,
"E1": 41,
"F1": 44,
"FS1": 46,
"G1": 49,
"GS1": 52,
"A1": 55,
"AS1": 58,
"B1": 62,
"C2": 65,
"CS2": 69,
"D2": 73,
"DS2": 78,
"E2": 82,
"F2": 87,
"FS2": 93,
"G2": 98,
"GS2": 104,
"A2": 110,
"AS2": 117,
"B2": 123,
"C3": 131,
"CS3": 139,
"D3": 147,
"DS3": 156,
"E3": 165,
"F3": 175,
"FS3": 185,
"G3": 196,
"GS3": 208,
"A3": 220,
"AS3": 233,
"B3": 247,
"C4": 262,
"CS4": 277,
"D4": 294,
"DS4": 311,
"E4": 330,
"F4": 349,
"FS4": 370,
"G4": 392,
"GS4": 415,
"A4": 440,
"AS4": 466,
"B4": 494,
"C5": 523,
"CS5": 554,
"D5": 587,
"DS5": 622,
"E5": 659,
"F5": 698,
"FS5": 740,
"G5": 784,
"GS5": 831,
"A5": 880,
"AS5": 932,
"B5": 988,
"C6": 1047,
"CS6": 1109,
"D6": 1175,
"DS6": 1245,
"E6": 1319,
"F6": 1397,
"FS6": 1480,
"G6": 1568,
"GS6": 1661,
"A6": 1760,
"AS6": 1865,
"B6": 1976,
"C7": 2093,
"CS7": 2217,
"D7": 2349,
"DS7": 2489,
"E7": 2637,
"F7": 2794,
"FS7": 2960,
"G7": 3136,
"GS7": 3322,
"A7": 3520,
"AS7": 3729,
"B7": 3951,
"C8": 4186,
"CS8": 4435,
"D8": 4699,
"DS8": 4978
}


def playtone(note_):
    R.power_board.piezo.buzz(0.13, note_)
def play(mysong):
    for i in range(len(mysong)):
        if (mysong[i] != 0):
            playtone((tones[mysong[i]])-50)
        else:
            time.sleep(0.25)


while True:
    R.wait_start()


    for i in range(3):
        R.power_board.piezo.buzz(0.1, Note.D6)#start
        time.sleep(0.1)

    with open('instructions.txt', 'r')as file_:
        instructions = file_.readlines()

    for i in range(len(instructions)):
        instructions[i] = instructions[i].split(',')
    for i in range(len(instructions)):
        if instructions[i][0] == 'forward':
            time_, power = move((float(instructions[i][1])*2))
            motor(1,0)
            motor(-1,1)
            time.sleep(time_)
            motor(BRAKE,0)
            motor(BRAKE,1)
        if instructions[i][0] == 'turn':
            turn_angle(int(instructions[i][1]))
    R.power_board.piezo.buzz(0.1, Note.D6)
    time.sleep(0.1)
    R.power_board.piezo.buzz(0.1, Note.D6)#end



    song = ['E7','E7', 0,'E7', 0,'C7','E7', 0,'G7', 0, 0, 0,'G6', 0, 0, 0,'C7', 0, 0,'G6', 0, 0,'E6', 0, 0,'A6', 0,'B6', 0,'AS6','A6', 0,'G6','E7','G7', 'A7', 0,'F7','G7', 0,'E7', 0,'C7','D7','B6', 0, 0,'C7', 0, 0,'G6', 0, 0,'E6', 0, 0,'A6', 0,'B6', 0,'AS6','A6', 0,'G6','E7','G7','A7', 0,'F7','G7', 0,'E7', 0,'C7','D7','B6', 0, 0]          
    play(song)

##for i in range(-100,100):
##    motors(i/100)
##    #servos(i/100)
##    time.sleep(0.05)

##motor(1,0)
##motor(-1,1)
##time.sleep(3)
##motor(0,0)
##motor(0,1)

##turn_angle(90)
##motor(-1,0)
##motor(1,1)
##time.sleep(3)
##motor(0,0)
##motor(0,1)



##marker_ids = R.camera.save(R.usbkey / "initial-view.png")
##markers = R.camera.see_ids()
##
##for i in markers:
##    print(i,'\n\n\n')
##
##file = open('markers.txt', 'w')
##markers.append('----------------')
##for i in markers:
##    file.write(str(i))
##file.close()
