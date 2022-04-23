import math
from robot_module import RobotModule
import time


class Vector3D:

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "({}, {}, {})".format(self.x, self.y, self.z)

    # TODO: add vector maths methods


class Position3D(Vector3D):

    def __init__(self, x, y, z):
        super().__init__(x, y, z)

    def none():
        return Position3D(None, None, None)

    def __str__(self):
        return "({}, {}, {})".format(self.x, self.y, self.z)


class MapMarker():

    def __init__(self,
                 id,
                 real_position,
                 real_orientation,
                 measured_data=None):
        self.id = id
        self.real_position = real_position
        self.measured_data = measured_data

    def mark_as_seen(self):
        self.can_see = True

    def mark_as_unseen(self):
        self.can_see = False


class MapCan():

    def __init__(self,
                 real_position,
                 correct_orientation,
                 detected=False,
                 last_detected=-1,
                 picked_up=False):
        self.real_position = real_position
        self.correct_orientation = correct_orientation
        self.detected = detected
        self.last_detected = last_detected
        self.picked_up = picked_up

    def mark_as_picked_up(self):
        self.picked_up = True
        self.detected = False
        self.last_detected = -1

    def mark_as_detected(self):
        self.detected = True
        self.last_detected = time.time()


class MapMananger(RobotModule):

    def __init__(self, map_width, map_length, cans, markers, robot_position,
                 robot_orientation):
        self.map_width = map_width
        self.map_length = map_length
        self.cans = cans
        self.markers = markers
        self.robot_position = robot_position
        self.robot_orientation = robot_orientation

    def update_robot_position(self, robot_position, robot_orientation):
        self.robot_position = robot_position
        self.robot_orientation = robot_orientation

    def update_market_perception(self, marker_id, data):
        for marker in self.markers:
            if marker.id == marker_id:
                marker.measured_data = data

    def infer_robot_position(self):
        # This involves alot of complex maths i dont understand but it works
        for marker in self.markers:
            if marker.can_see:
                # We know the real position of this marker, the distance and angle to the camera
                # Draw a line from the markers real position at the angle distance of the camera
                line_start = marker.real_position
                line_end = Vector2D(
                    line_start.x + marker.measured_data.distance *
                    math.cos(marker.measured_data.spherical.rot_x),
                    line_start.y + marker.measured_data.distance *
                    math.sin(marker.measured_data.angle), 0)
