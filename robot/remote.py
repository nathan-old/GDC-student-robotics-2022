# PREFACE:
# This module is ONLY used for debuging, it will NOT run in comp mode
# and is purley there to test movement, features or aspect of the code
# It allows remote controll of the robot, via a rc controller connected
# to the ruggiduino. As previously stated, this is ONLY used for debugging
# purposes and the rc controller will be removed from the robot before the
# competition.

import sr.robot3 as sr
from robot_module import RobotModule
import time, serial, numpy, threading


def range_map(value, max_, min_, ranges):
    return round((ranges[0] + (ranges[1] - ranges[0]) * ((value - min_) /
                                                         (max_ - min_))), 2)


class RemoteController(RobotModule):

    def __init__(
        self,
        robot,
        movementManager,
        ruggiduino_id,
        serial_port,
        serial_baud_rate,
        shutdown_callback
    ):
        self.robot = robot
        self.ruggiduino_id = ruggiduino_id
        self.serial_port = serial_port
        self.serial_baud_rate = serial_baud_rate
        self.movementManager = movementManager

        self.serial = None
        self.ruggeduino_device = self.robot.ignored_ruggeduinos[
            self.ruggiduino_id]
        self.shutdown_callback = shutdown_callback
        self.running = False

    def start_serial(self):
        if self.running or self.serial is not None:
            print(
                "Tried to start serial, but it's already running (running={}, serial is None={})"
                .format(self.running, self.serial is None))
            return

        self.serial = serial.Serial(self.serial_port, self.serial_baud_rate)

    def start_listening(self):
        if (self.ruggiduino_device is None):
            print(
                "Tried to run RemoteController, but ruggiduino_device is None")
            return
        elif (self.serial is None):
            print("Tried to run RemoteController, but serial is None")
            return
        elif (self.running):
            print("Tried to run RemoteController, but already running")
            return

        self.running = True
        self.thread_handle = threading.Thread(target=self.run_thread)

    def run_thread(self):
        x = numpy.array([[-0.33, 0.58, 0.33], [-0.33, -0.58, 0.33],
                         [0.67, 0, 0.33]])
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

        while self.running:
            arr = self.serial.readline().decode().rstrip().split(',')

            x_value = int(
                arr[0])
            y_value = int(arr[1])
            r_value = int(arr[2])
            s_value = int(arr[3])
            b_value = int(arr[4]) 
            t_value = int(arr[5]) 
            shutdown = bool(arr[6])
            if shutdown:
                self.running = False
                self.serial.close()
                self.serial = None
                self.shutdown_callback()
                return
                
            if s_value < 1500:
                s_state = True
            else:
                s_state = False
            if b_value > 1500:
                b_state = True
            else:
                b_state = False

            x_value = range_map(x_value, x_max, x_min, [-1, 1])
            y_value = range_map(y_value, y_max, y_min, [-1, 1])
            r_value = range_map(r_value, r_max, r_min, [-1, 1])
            t_value = range_map(t_value, t_max, t_min, [0, 1])

            if s_state == True:
                t_value *= -1

            y = numpy.array([[x_value], [t_value], [r_value]])

            # Get the dot product of the array and the RC input and then bound between -1 and 1
            A_speed = min(max((round(numpy.dot(x, y)[0][0], 1)), 1), -1)
            B_speed = min(max((round(numpy.dot(x, y)[1][0], 1)), 1), -1)
            C_speed = min(max((round(numpy.dot(x, y)[2][0], 1)), 1), -1)

            # apply this to the motors
            # TODO: move to using the move, turn angle, etc commands instead of the raw speed
            self.movementManager.set_speed(A_speed, 0)
            self.movementManager.set_speed(B_speed, 1)
            self.movementManager.set_speed(C_speed, 2)
            self.movementManager.wait_for_queue_clearance()
            time.sleep(self.wait_between_reads)

        print("Stopping RemoteController")
        self.serial.close()