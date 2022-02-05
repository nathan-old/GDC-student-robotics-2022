from sr.robot3 import Robot
from sr.robot3.env import CONSOLE_ENVIRONMENT_WITH_VISION
R = Robot(env=CONSOLE_ENVIRONMENT_WITH_VISION, wait_start=True)

def turn(pos):
    R.servo_boards["sr0GG39"].servos[0].position = pos
def map_(x):
    return (x/180*2)-1
R.wait_start()
while True:
    turn(0)
    time.sleep(2)
    turn(-1)
    time.sleep(2)