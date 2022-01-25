import math, time
from robot_module import RobotModule
from threading import Thread, RLock


class MovementCommand:

    def __init__(self,
                 command,
                 rpm=None,
                 distance=None,
                 angle=None,
                 power=None,
                 motor_index=None):
        if (command == "move"):
            self.code = 0
            self.rpm = rpm
            self.distance = distance
        elif (command == "turn_angle"):
            self.code = 1
            self.angle = angle
        elif (command == "power"):
            self.code = 2
            self.motor_index = motor_index
            self.power = power
        else:  # TODO: add helper command like forward that uses sensors.forward() to orient the robot and then move forward distance amount
            raise Exception("Invalid command")

    def execute(self, M_Master):
        if (self.code == 0):
            M_Master._move(self.rpm, self.distance)
        elif (self.code == 1):
            M_Master._turn_angle(self.angle)
        elif (self.code == 2):
            M_Master._set_power(self.power, self.motor_index)
        else:
            raise Exception("Invalid command")
        return True


class MovementController(RobotModule):

    def __init__(self,
                 robot,
                 markerManager,
                 wheel_circumference=0.15,
                 arm_radius=0.18,
                 wrap_angles=False,
                 queue_max_length=30,
                 time_between_commands=0,
                 loop_delay=0.1,
                 motor_boards=['SR0PJ1W', 'SR0VG1M']):
        super().__init__("MovementController", "Movement controller", "0.0.1",
                         "0.0.0", "1.0.0")
        self.__M_Master = MotorMaster(robot, wheel_circumference, arm_radius,
                                      wrap_angles, motor_boards)
        self.markerManager = markerManager
        self.queue_max_length = queue_max_length
        self.time_between_commands = time_between_commands
        self.loop_delay = loop_delay
        self.lock = RLock()
        self.queue = []
        self.cleaned = True
        self.running = False

    def start_loop(self):
        self.running = True
        return Thread(target=self.__loop).start()

    def shutdown(self):
        self.running = False

    def __loop(self):
        while self.running:
            self.execute_queue()
            time.sleep(self.loop_delay)

    def set_motor_master(self, motor_master):
        self.__M_Master = motor_master

    def set_queue_max_length(self, queue_max_length):
        self.queue_max_length = queue_max_length

    def add_to_queue(self, command):
        with self.lock:
            if len(self.queue) >= self.queue_max_length:
                self.queue.pop(0)
            self.queue.append(command)

    def execute_queue(self):
        with self.lock:
            self.queue[:] = [
                x for x in self.queue if not x.execute(self.__M_Master)
            ]
            self.cleaned = len(self.queue) == 0

    def move(self, distance, rpm=140):
        with self.lock:
            self.queue.append(MovementCommand("move", rpm, distance, None))

    def turn_angle(self, angle):
        with self.lock:
            self.queue.append(MovementCommand("turn_angle", None, None, angle))

    def set_power(self, power, index):
        with self.lock:
            self.queue.append(
                MovementCommand("power", None, None, None, power, index))

    def forward(self, rpm):
        self.move(1, rpm)

    def wait_for_queue_clearance(self):
        while not self.cleaned:
            time.sleep(0.05)


class MotorMaster():

    def __init__(self,
                 robot,
                 wheel_circumference=0.3299,
                 arm_radius=0.28,
                 wrap_angles=False,
                 motor_boards=['SR0PJ1W', 'SR0VG1M']):
        self.robot = robot
        self.wheel_circumference = wheel_circumference
        self.wheel_A = self.robot.motor_boards[motor_boards[0]].motors[0]
        self.wheel_B = self.robot.motor_boards[motor_boards[0]].motors[1]
        self.wheel_B = self.robot.motor_boards[motor_boards[1]].motors[0]
        self.arm_radius = arm_radius
        self.wrap_angles = wrap_angles
        self.wheels = [self.wheel_A, self.wheel_B, self.wheel_C]
        # brake by default
        for wheel in self.wheels:
            wheel.power = 0

    # YOU SHOULD NOT BE CALLING THESE (PEP 8 - https://www.python.org/dev/peps/pep-0008/#method-names-and-instance-variables)
    # YOU SHOULD BE CALLING THE QUEUE METHODS ON MovementController INSTEAD
    def _set_power(self, power=0, wheel_index=None):
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
        speed = self.wheel_circumference * rps  # in m/s
        time_ = float(distance / speed)  # in secconds
        return time_, power

    def _move(self, x_distance, rpm=140):
        distance = x_distance/math.cos(math.radins(30))
        time_, power = self.__calculate_move(distance, rpm)
        print("Moving " + str(distance) + " meters, " + str(time_) +
              " seconds, at " + str(rpm) + " rpm, with power " + str(power))
        self._set_power(power)
        time.sleep(time_)
        self._set_power(0)

    def _turn_angle(self, angle):
        if angle > 180 and self.wrap_angles:
            angle = 180 - angle
        distance = angle * ((math.pi * 2 * self.arm_radius) / 360)
        print("Turning " + str(angle) + " degrees, " + str(distance) +
              " meters")
        self._move(distance)
