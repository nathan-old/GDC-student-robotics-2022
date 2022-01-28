import time
from robot_module import RobotModule
from threading import Thread
from datetime import datetime


class MarkerManager(RobotModule):

    def __init__(self, robot, wait_between_reads=0.1, debug_markers=True):
        super().__init__(
            "MarkerManager",
            "Processess the markers and attempts to locate the robot, which is then passed to the mapper",
            "1.0.0", "0.0.0", "1.0.0")

        self.robot = robot
        self.camera = robot.camera

        self.running = False
        self.debug_markers = debug_markers
        self.seen_markers = []
        self.wait_between_reads = wait_between_reads
        self.thread = None
        self.save_count = 0

    def see_markers(self):
        dt = datetime.now()
        p = str(self.robot.usbkey) + "/photos/{}-{}.jpg".format(
            dt.day,self.save_count)
       
        self.camera.save(p)
        self.seen_markers = self.camera.see()
        if len(self.seen_markers) != 0:
            print("Can see {} markers".format(len(self.seen_markers)))
        self.save_count += 1

    def read_wait(self):
        time.sleep(self.wait_between_reads)

    def shutdown(self):
        self.running = False

    def start_loop(self):
        self.running = True
        self.thread = Thread(target=self.loop).start()

    def loop(self):
        while self.running:
            self.see_markers()
            # TODO: Process markers and determine postion, then update self.mapperManagers "RobotPosition"
            for marker in self.seen_markers:
                print(
                    "({}) distance: {} meters, rot: ({}, {}) (XY L->R), spherical distance: {} meters, cartesian position: ({}, {}, {}) (XYZ)"
                    .format(marker.id, marker.distance, marker.spherical.rot_x,
                            marker.spherical.rot_y, marker.spherical.dist,
                            marker.cartesian.x, marker.cartesian.y,
                            marker.cartesian.z))
            self.read_wait()
        print("Camera loop no longer running")
