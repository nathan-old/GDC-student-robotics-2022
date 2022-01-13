from sr.robot3 import *
import time
print ("hello world")
R = Robot(auto_start=True)
R._log_discovered_boards
motor_0 = R.motor_board.motors[0]
motor_1 = R.motor_board.motors[1]

R.wait_start()
R.servo_board.servos[1].position = -1
motor_0.power = 0.5
motor_1.power = 0.5
time.sleep(3)
motor_0.power = 1
motor_1.power = 1
R.servo_board.servos[1].position = 1

for i in range(-100,100):
    R.servo_board.servos[1].position = (i/100)
R.power_board.piezo.buzz(2, 400)
for i in range(50):
    markers = R.camera.see()
    print("I can see", len(markers), "markers:")

    for m in markers:
        print(" - Marker #{0} is {1} metres away".format(m.id, m.distance))#giving completely unresonable values
        
R.power_board.piezo.buzz(2, 400)
