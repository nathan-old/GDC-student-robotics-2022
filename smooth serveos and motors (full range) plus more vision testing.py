from sr.robot3 import *
import time
print ("hello world")
R = Robot(auto_start=True, verbose=True)
R._log_discovered_boards
def motors(pow):
    R.motor_board.motors[0].power = pow
    R.motor_board.motors[1].power = pow
def servos(pos):
    R.servo_board.servos[0].position = pos

R.wait_start()

for i in range(-100,100):
    motors(i/100)
    servos(i/100)
    time.sleep(0.05)
motors(BRAKE)

marker_ids = R.camera.save(R.usbkey / "initial-view.png")
markers = R.camera.see_ids()

for i in markers:
    print(i,'\n\n\n')