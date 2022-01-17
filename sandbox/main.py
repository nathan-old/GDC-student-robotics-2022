#%%
import math
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import matplotlib as mpl


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


class Map():

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
                line_end = Vector3D(
                    line_start.x + marker.measured_data.distance *
                    math.cos(marker.measured_data.spherical.rot_x),
                    line_start.y + marker.measured_data.distance *
                    math.sin(marker.measured_data.angle), 0)

    def plot(self):
        # Create figure and axes
        fig, ax = plt.subplots()
        xpoints = np.array([0, self.map_width])
        ypoints = np.array([0, self.map_length])

        plt.plot(xpoints, ypoints, color='None')

        # Create a Rectangle patch
        walls = patches.Rectangle((0, 0),
                                  self.map_width,
                                  self.map_length,
                                  linewidth=1,
                                  edgecolor='black',
                                  facecolor='none')
        raised_platform = patches.Rectangle(
            ((self.map_width / 2) - 600, (self.map_length / 2) - 600),
            1200,
            1200,
            linewidth=1,
            facecolor='grey')

        sz1 = plt.Polygon(np.array([[0, 0], [0, 2500], [2500, 0]]), color='r')
        sz2 = plt.Polygon(np.array([[0, self.map_length],
                                    [0, self.map_length - 2500],
                                    [2500, self.map_length]]),
                          color='b')
        sz3 = plt.Polygon(np.array([[self.map_width, 0],
                                    [self.map_width, 2500],
                                    [self.map_width - 2500, 0]]),
                          color='g')
        sz4 = plt.Polygon(np.array([[self.map_width, self.map_length],
                                    [self.map_width, self.map_length - 2500],
                                    [self.map_width - 2500, self.map_length]]),
                          color='y')

        start_zone_1 = patches.Rectangle((0, 0),
                                         1000,
                                         1000,
                                         linewidth=1,
                                         facecolor='white')

        start_zone_2 = patches.Rectangle((0, self.map_length - 1000),
                                         1000,
                                         1000,
                                         linewidth=1,
                                         facecolor='white')

        start_zone_3 = patches.Rectangle((self.map_width - 1000, 0),
                                         1000,
                                         1000,
                                         linewidth=1,
                                         facecolor='white')

        start_zone_4 = patches.Rectangle(
            (self.map_width - 1000, self.map_length - 1000),
            1000,
            1000,
            linewidth=1,
            facecolor='white')

        ax.add_patch(sz1)
        ax.add_patch(sz2)
        ax.add_patch(sz3)
        ax.add_patch(sz4)

        ax.add_patch(start_zone_1)
        ax.add_patch(start_zone_2)
        ax.add_patch(start_zone_3)
        ax.add_patch(start_zone_4)

        ax.add_patch(walls)
        ax.add_patch(raised_platform)

        for can in self.cans:
            can_obj = patches.Circle((0, 0), 67, color='r')
            ax.add_patch(can_obj)
        for marker in self.markers:
            marker_obj = patches.Circle(
                (marker.real_position.x, marker.real_position.z),
                50,
                color='black')
            ax.add_patch(marker_obj)

        # Plot the robot
        r = 250
        dt = plt.Polygon(
                np.array([
                    [
                        self.robot_position.x + (-0.866 * r),
                        self.robot_position.y + (0.5 * r)
                    ],
                    [
                        self.robot_position.x + (0.866 * r),
                        self.robot_position.y + (0.5 * r)
                    ],
                    [
                        self.robot_position.x,
                        self.robot_position.y + (1 * r)
                    ],
                ]))

        r2 = plt.Polygon(
                np.array([
                    [
                        self.robot_position.x + (-0.866 * r),
                        self.robot_position.y + (0.5 * r)
                    ],
                    [
                        self.robot_position.x + (0.866 * r),
                        self.robot_position.y + (0.5 * r)
                    ],
                    [
                        self.robot_position.x,
                        self.robot_position.y + (1 * r)
                    ],
                ]))
        t2 = mpl.transforms.Affine2D().rotate_deg(self.robot_orientation) + ax.transData
        r2.set_transform(t2)

        ax.add_patch(dt)
        ax.add_patch(r2)

        plt.show()


MAP_HEIGHT = 5750
MAP_WIDTH = 5750
markers = [
    MapMarker("o", Position3D(0, 0, 718), None),
    MapMarker("o", Position3D(0, 0, (718 * 2)), None),
    MapMarker("o", Position3D(0, 0, (718 * 3)), None),
    MapMarker("o", Position3D(0, 0, (718 * 4)), None),
    MapMarker("o", Position3D(0, 0, (718 * 5)), None),
    MapMarker("o", Position3D(0, 0, (718 * 6)), None),
    MapMarker("o", Position3D(0, 0, (718 * 7)), None),
    MapMarker("o", Position3D(MAP_WIDTH, 0, 718), None),
    MapMarker("o", Position3D(MAP_WIDTH, 0, (718 * 2)), None),
    MapMarker("o", Position3D(MAP_WIDTH, 0, (718 * 3)), None),
    MapMarker("o", Position3D(MAP_WIDTH, 0, (718 * 4)), None),
    MapMarker("o", Position3D(MAP_WIDTH, 0, (718 * 5)), None),
    MapMarker("o", Position3D(MAP_WIDTH, 0, (718 * 6)), None),
    MapMarker("o", Position3D(MAP_WIDTH, 0, (718 * 7)), None),
    MapMarker("o", Position3D(718, 0, MAP_WIDTH), None),
    MapMarker("o", Position3D((718 * 2), 0, MAP_WIDTH), None),
    MapMarker("o", Position3D((718 * 3), 0, MAP_WIDTH), None),
    MapMarker("o", Position3D((718 * 4), 0, MAP_WIDTH), None),
    MapMarker("o", Position3D((718 * 5), 0, MAP_WIDTH), None),
    MapMarker("o", Position3D((718 * 6), 0, MAP_WIDTH), None),
    MapMarker("o", Position3D((718 * 7), 0, MAP_WIDTH), None),
    MapMarker("o", Position3D(718, 0, 0), None),
    MapMarker("o", Position3D((718 * 2), 0, 0), None),
    MapMarker("o", Position3D((718 * 3), 0, 0), None),
    MapMarker("o", Position3D((718 * 4), 0, 0), None),
    MapMarker("o", Position3D((718 * 5), 0, 0), None),
    MapMarker("o", Position3D((718 * 6), 0, 0), None),
    MapMarker("o", Position3D((718 * 7), 0, 0), None),
]
map = Map(MAP_WIDTH, MAP_HEIGHT, [], markers, Position3D(500, 0, 500), 90)
while map.robot_position.y < map.map_length - 100:
    map.plot()
    map.robot_orientation += 5
    time.sleep(0.25)

# %%
