import math, time
from robot_module import RobotModule
from threading import Thread
class MovementCommand:
    def __init__(self, command, rpm, distance, angle):
        if (command == "move"):
            self.code = 0
            self.rpm = rpm
            self.distance = distance
        elif (command == "turn_angle"):
            self.code = 1
            self.rpm = rpm
            self.angle = angle
        else: # TODO: add helper command like forward that uses sensors.forward() to orient the robot and then move forward distance amount
            raise Exception("Invalid command")

    def execute(self, __M_Master):
        if (self.code == 0):
            __M_Master.__move(self.rpm, self.distance)
        elif (self.code == 1):
            __M_Master.__turn_angle(self.rpm, self.angle)
        else:
            raise Exception("Invalid command")

class MovementController(RobotModule):
    def __init__(self, robot, wheel_circumference=0.15, arm_radius=0.18, wrap_angles=False, queue_max_length = 30, time_between_commands = 0, loop_delay = 0.1):
        super.__init__("MovementController", "Movement controller", "0.0.1", "0.0.0", "1.0.0")
        self.__M_Master = MotorMaster(robot, wheel_circumference, arm_radius, wrap_angles)
        self.queue_max_length = queue_max_length
        self.time_between_commands = time_between_commands
        self.loop_delay = loop_delay
        self.queue = []

    def start_loop(self):
        return Thread(target=self.__loop).start()

    # TODO: handle a way to shutdown the thread
    def __loop(self):
        while True:
            self.execute_queue()
            time.sleep(self.loop_delay)
    
    def set_motor_master(self, motor_master):
        self.__M_Master = motor_master
    
    def set_queue_max_length(self, queue_max_length):
        self.queue_max_length = queue_max_length
    
    def add_to_queue(self, command):
        if len(self.queue) >= self.queue_max_length:
            self.queue.pop(0)
        self.queue.append(command)
    
    def execute_queue(self):
        self.queue[:] = [x for x in self.queue if not x.execute(self.__M_Master)]

    def move(self, distance, rpm = 140):
        self.queue.append(MovementCommand("move", rpm, distance))

    def turn_angle(self, angle):
        self.queue.append(MovementCommand("turn_angle", angle))
    
    def wait_for_queue_clearance(self):
        while len(self.queue) > 0:
            time.sleep(0.05)
        

class MotorMaster():
    def __init__(self, robot, wheel_circumference=0.15, arm_radius=0.18, wrap_angles=False):
        self.robot = robot
        self.wheel_circumference = wheel_circumference
        self.wheel_one = self.robot.motor_boards['SR0PJ1W'].motors[0]
        self.wheel_two = self.robot.motor_boards['SR0PJ1W'].motors[1]
        self.wheel_three = self.robot.motor_boards['SR0VG1M'].motors[0] # Can we not put the third motor on the same board as 1 & 2?
        self.arm_radius = arm_radius
        self.wrap_angles = wrap_angles
        self.wheels = [self.wheel_one, self.wheel_two, self.wheel_three]
        # brake by default
        for wheel in self.wheels:
            wheel.power = 0

    
    # YOU SHOULD NOT BE CALLING THESE (PEP 8 https://www.python.org/dev/peps/pep-0008/#method-names-and-instance-variables)
    # YOU SHOULD BE CALLING THE METHODS ON MovementController INSEAD
    def __set_power(self, power=0, wheel_index=None):
        if (wheel_index == None):
            for wheel in self.wheels:
                wheel.power = power
        else:
            self.wheels[wheel_index].power = power

    def __calculate_move(self, distance, rpm):
        # if our distance is negative, invert it and the power
        power = 1 if distance > 0 else 0
        distance = distance if distance > 0 else distance * -1

        rps = rpm / 60
        speed = self.wheel_circumference * rps # in m/s
        time_ = float(distance/speed) # in secconds
        return time_, power

    def __move(self, distance, rpm = 140):
        time_, power = self.calculate_move(distance, rpm) 
        self.set_power(power)
        time.sleep(time_)
        self.set_power(0)

    def __turn_angle(self, angle):
        if angle > 180 and self.wrap_angles:
           angle = 180 - angle
        distance = angle * ((math.pi * 2 * self.arm_radius) / 360)
        self.move(distance)


