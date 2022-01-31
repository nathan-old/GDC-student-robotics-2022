# %%
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time, matplotlib
import math, random, functools

matplotlib.use('Agg')


class Vector2D():

    def __init__(self, x, y):
        self._x = x
        self._y = y

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y


class Point():

    def __init__(self, x, y, color, label):
        self._x = x
        self._y = y
        self._color = color
        self._label = label

    @property
    def position(self):
        return Vector2D(self._x, self._y)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y


class Label():

    def __init__(self, text, color, offset=Vector2D(0, 0)):
        self._text = text
        self._color = color
        self._offset = offset

    @property
    def text(self):
        return self._text

    @property
    def color(self):
        return self._color

    @property
    def offset(self):
        return self._offset


class Line():

    def __init__(self, start, end, color, linestyle='solid', linewidth=1):
        self._start = start
        self._end = end
        self._color = color
        self._linestyle = linestyle
        self._linewidth = linewidth

    @property
    def start(self):
        return self._start

    @property
    def end(self):
        return self._end

    @property
    def color(self):
        return self._color

    @property
    def linestyle(self):
        return self._linestyle

    @property
    def linewidth(self):
        return self._linewidth


class Marker():

    def __init__(self, id, x, y, can_see=False):
        self._position = Vector2D(x, y)
        self._id = id
        self._can_see = can_see

    @property
    def position(self):
        return self._position

    @property
    def can_see(self):
        return self._can_see


class Map():

    def __init__(self, central_box, arena_border, can_list, point_zone,
                 start_boxes, marker_list):
        self.central_box = central_box
        self.arena_border = arena_border
        self.can_list = can_list
        self.point_zone = point_zone
        self.start_boxes = start_boxes
        self.marker_list = marker_list

    def move_can(self, can_index, new_pos):
        if can_index > len(self.can_list) - 1:
            raise Exception("Can index out of bounds")
        if type(new_pos) != Vector2D:
            raise Exception("New position must be a vector")
        self.can_list[can_index] = new_pos

    def set_can_see_marker(self, marker_id, can_see):
        if marker_id > len(self.marker_list) - 1:
            raise Exception("Marker index out of bounds")

        self.marker_list[marker_id].can_see = can_see

    # Renders the background image from the static elements
    def render_static(self):
        # Draw start boxes/zones
        lines = []
        points = []
        for i in range(len(self.start_boxes) - 1):
            lines.append(Line(self.start_boxes[i], self.start_boxes[i + 1],
                              ''))

        # Draw central raised platform
        for i in range(len(self.central_box) - 1):
            lines.append(Line(self.central_box[i], self.central_box[i + 1],
                              ''))

        # Draw point zones
        for i in range(len(self.point_zone)):
            lines.append(
                Line(self.point_zone[i], self.point_zone[i], 'r', '--'))

        # Draw arena border
        for i in range(len(self.arena_border)):
            lines.append(Line(self.arena_border[i], self.arena_border[i], 'b'))

        # Draw cans
        for i in range(len(can_list)):
            points.append(Point(can_list[i].x, can_list[i].y, 'r', 'i'))

        return [lines, points]
        # load the precreated background.png
        #    img = plt.imread("background.png")
        #   fig, ax = plt.subplots()
        #  ax.imshow(img, extent=[0, 5750, 0, 5750], aspect='auto', zorder=0)

    # Renders the dynamic elements (cans, markers)
    def render_dynamic(self):
        # Draw markers
        lines = []
        points = []
        for marker in self.marker_list:
            points.append(
                Point(marker.x, marker.y, 'g' if marker.can_see else 'r'))

        for i in range(len(points)):
            if i < 7:
                points[i]._label = Label(str(i), 'w', Vector2D(0, -40))
            elif 6 < i < 14:
                points[i]._label = Label(str(i), 'w', Vector2D(0, -60))
            elif 13 < i < 21:
                points[i]._label = Label(str(i), 'w', Vector2D(0, +40))
            elif 20 < i < 28:
                points[i]._label = Label(str(i), 'w', Vector2D(0, +60))
            else:
                points[i]._label = Label(str(i), 'w')

        def set_static(self, figure):
            self.use_static = True

            figure.savefig(fname="background.png",
                           dpi=500,
                           transparent=True,
                           bbox_inches='tight',
                           pad_inches=0)
            plt.close(figure)


central_box = [
    Vector2D(2275, 2275),
    Vector2D(2275, 3475),
    Vector2D(3475, 3475),
    Vector2D(3475, 2275),
    Vector2D(2275, 2275)
]
arena_border = [
    Vector2D(0, 0),
    Vector2D(0, 5750),
    Vector2D(5750, 5750),
    Vector2D(5750, 0),
    Vector2D(0, 0)
]
can_list = [
    Vector2D(2875, 33.5),
    Vector2D(33.5, 2875),
    Vector2D(2875, 5716.5),
    Vector2D(5716.5, 2875),
    Vector2D(900, 900),
    Vector2D(4850, 900),
    Vector2D(4850, 4850),
    Vector2D(900, 4850),
    Vector2D(1135, 2875),
    Vector2D(2875, 1135),
    Vector2D(4615, 2875),
    Vector2D(2875, 4615)
]
point_zone = [
    Vector2D(2500, 0),
    Vector2D(3250, 0),
    Vector2D(5750, 2500),
    Vector2D(5750, 3250),
    Vector2D(3250, 5750),
    Vector2D(2500, 5750),
    Vector2D(0, 3250),
    Vector2D(0, 2500),
    Vector2D(2500, 0)
]
start_boxes = [
    Vector2D(0, 1000),
    Vector2D(1000, 1000),
    Vector2D(1000, 0),
    Vector2D(4750, 0),
    Vector2D(4750, 1000),
    Vector2D(5750, 1000),
    Vector2D(5750, 4750),
    Vector2D(4740, 4750),
    Vector2D(4750, 5750),
    Vector2D(1000, 5750),
    Vector2D(1000, 4750),
    Vector2D(0, 4750),
    Vector2D(0, 1000)
]

marker_list = [
    Marker(0, 718.75, 5750),
    Marker(1, 1437.5, 5750),
    Marker(2, 2156.25, 5750),
    Marker(3, 2875, 5750),
    Marker(4, 3593.75, 5750),
    Marker(5, 4312.5, 5750),
    Marker(6, 5031.25, 5750),
    Marker(7, 5750, 5031.25),
    Marker(8, 5750, 4312.5),
    Marker(9, 5750, 3593.75),
    Marker(10, 5750, 2875),
    Marker(11, 5750, 2156.25),
    Marker(12, 5750, 1437.5),
    Marker(13, 5750, 718.75),
    Marker(14, 5031.25, 0),
    Marker(15, 4312.5, 0),
    Marker(16, 3593.75, 0),
    Marker(17, 2875, 0),
    Marker(18, 2156.25, 0),
    Marker(19, 1437.5, 0),
    Marker(20, 718.75, 0),
    Marker(21, 0, 718.75),
    Marker(22, 0, 1437.5),
    Marker(23, 0, 2156.25),
    Marker(24, 0, 2875),
    Marker(25, 0, 3593.75),
    Marker(26, 0, 4312.5),
    Marker(27, 0, 5031.25)
]

MAX_SPEED = 500  # Max speed (mm/s)
ROT_SPEED = 90  # degrees / s
FPS = 4
DURATION = 10  # secconds


def point_in_triangle(point, triangle):
    """Returns True if the point is inside the triangle
    and returns False if it falls outside.
    - The argument *point* is a tuple with two elements
    containing the X,Y coordinates respectively.
    - The argument *triangle* is a tuple with three elements each
    element consisting of a tuple of X,Y coordinates.

    It works like this:
    Walk clockwise or counterclockwise around the triangle
    and project the point onto the segment we are crossing
    by using the dot product.
    Finally, check that the vector created is on the same side
    for each of the triangle's segments.
    """
    # Unpack arguments
    x, y = point
    ax, ay = triangle[0]
    bx, by = triangle[1]
    cx, cy = triangle[2]
    # Segment A to B
    side_1 = (x - bx) * (ay - by) - (ax - bx) * (y - by)
    # Segment B to C
    side_2 = (x - cx) * (by - cy) - (bx - cx) * (y - cy)
    # Segment C to A
    side_3 = (x - ax) * (cy - ay) - (cx - ax) * (y - ay)
    # All the signs must be positive or all negative
    return (side_1 < 0.0) == (side_2 < 0.0) == (side_3 < 0.0)


def get_intersections(x0, y0, r0, x1, y1, r1):
    d = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    if d > r0 + r1:
        return [[-1, -1], [-1, -1]]
    if d < abs(r0 - r1):
        return [[-2, -2], [-2, -2]]
    if d == 0 and r0 == r1:
        return [[-3, -3], [-3, -3]]
    else:
        a = (r0**2 - r1**2 + d**2) / (2 * d)
        h = math.sqrt(r0**2 - a**2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d
        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d
        return ([[x3, y3], [x4, y4]])


CONFIG = {
    'camera': {
        'fov': 62,
        'range': 4000
    },
    'render': {
        'fps': FPS,
        'duration': DURATION,
        'display_mode': 'live',
    },
    'markers_list': marker_list,
}


class SimulatorCommand():

    def __init__(self, id, command, args):
        self.id = id
        self.command = command
        self.args = args


class SimulatorCamera():

    def __init__(self, sim, fov, range, marker_list, robot):
        self.sim = sim
        self.fov = fov
        self.range = range
        self.id = robot
        self.marker_list = marker_list
        self.can_see = []
        self.posed_markers = []

    @functools.cached_property
    def fov_points(self):
        x = self.robot.x
        y = self.robot.y

        endx = [
            x + self.range *
            math.cos(math.radians(self.robot.bearing.angle - self.fov / 2)),
            x + self.range *
            math.cos(math.radians(self.robot.bearing.angle + self.fov / 2))
        ]

        endy = [
            y + self.range *
            math.sin(math.radians(self.robot.bearing.angle - self.fov / 2)),
            y + self.range *
            math.sin(math.radians(self.robot.bearing.angle + self.fov / 2))
        ]
        return [endx, endy]

    def angle_to_point(self, x, y):
        robot_angle = self.robot.bearing.angle
        dx = x - self.robot.x
        dy = y - self.robot.y
        marker_angle = math.degrees(math.atan2(dx, dy))

        robot_angle -= 90
        if robot_angle < 0:
            robot_angle += 360
        robot_angle = 360 - robot_angle
        if robot_angle == 360:
            robot_angle = 0
        phi = (-1 * (marker_angle - robot_angle))
        if phi > 50:
            phi = phi - 360
        if phi < -30 or phi > 30:
            print("Calulated phi {} under -30 or over 30".format(phi))
        return phi + 90

    def distance_to_point(self, x, y):
        dx = x - self.robot.x
        dy = y - self.robot.y
        return math.sqrt(dx**2 + dy**2)

    def see_ids(self):
        self.can_see = []
        for marker in self.marker_list:
            id = marker[0]
            x = marker[1]
            y = marker[2]
            if point_in_triangle(
                (x, y), ((self.fov_points[0][0], self.fov_points[1][0]),
                         (self.robot.x, self.robot.y),
                         (self.fov_points[0][1], self.fov_points[1][1]))):
                self.can_see.append(id)

    def pose_estimation(self):
        # The values retruned from the "camera" in order theta, psi, r(distance), marker number
        self.posed_markers = []
        for id in self.can_see:
            self.posed_markers.append([
                90,
                self.angle_to_point(self.marker_list[id][1],
                                    self.marker_list[id][2]),
                self.distance_to_point(self.marker_list[id][1],
                                       self.marker_list[id][2]), id
            ])

    def see(self):
        self.see_ids()
        self.pose_estimation()
        return self.posed_markers


class SimulatorInterface():

    def __init__(self, parent, config):
        self.parent = parent
        self.config = config

    def _move(self, r, meters, speed=1):
        cmd = SimulatorCommand(r.id, 'power', [meters, speed])
        self.parent.update(cmd)

    def set_power(self, r, motor_id, power):
        cmd = SimulatorCommand(r.id, 'power', [motor_id, power])
        self.parent.update(cmd)

    def _rotate(self, r, degrees, speed=1):
        cmd = SimulatorCommand(r.id, 'rotate', [degrees, speed])
        self.parent.update(cmd)

    def stop(self, r):
        cmd = SimulatorCommand(r.id, 'stop', [])
        self.parent.update(cmd)

    def camera(self, r):
        return self.parent.get_camera(r.id)


class RobotInterface():

    def __init__(self, position, bearing, interface, id=0):
        self._pos = position
        self._bearing = bearing
        self._id = id
        self._interface = interface
        self.motors = [0, 0, 0]  # Power levels for each motor
        self.wheel_angles = [0, 0, 0]  # Angle of each wheel
        self.wheel_arm_lengths = [0, 0, 0]  # Length of each wheel arm
        # Choose random color for robot
        r = random.random()
        b = random.random()
        g = random.random()

        self._color = (r, g, b)

    @property
    def position(self):
        return Vector2D(self._pos.x, self._pos.y)

    @property
    def x(self):
        return self._pos.x

    @property
    def y(self):
        return self._pos.y

    @property
    def bearing(self):
        return self._bearing

    @property
    def id(self):
        return self._id

    @property
    def color(self):
        return self._color

    @property
    def interface(self):
        return self._interface

    def set_position(self, x, y):
        self._pos = Vector2D(x, y)

    def set_bearing(self, bearing):
        self._bearing = bearing

    def set_power(self, power, index=None):
        if index is None:
            for i in range(len(self.motors)):
                self.motors[i] = power

        else:
            if index >= len(self.motors):
                raise Exception("Motor index out of range")
            self.motors[index] = power

    def run_motors(self, time):
        # TODO: Calculate the new x,y position and rotation using: Current motor speeds, motor/wheel positon info, current position and rotation and time (which is in secconds)
        # This is probably done by creating a force vector, multiplying it by time applying it to the robot
        pass


class Bearing():

    def __init__(self, angle):
        self._angle = angle

    @property
    def angle(self):
        return self._angle

    def set_angle(self, angle):
        self._angle = angle

    def add_angle(self, angle):
        self._angle += angle
        if self._angle > 360:
            self._angle = self._angle - 360
        if self._angle < 0:
            self._angle = self._angle + 360


class Simulator():

    def __init__(self, config):
        self.config = config
        self.fps = config['render']['fps']
        self.duration = config['render']['duration']
        self.display_mode = config['render']['display_mode']
        self._interface = SimulatorInterface(self, config)
        self.robots = []
        self.controlled_robot = None
        self.updates = []

        self.running = False

    @property
    def interface(self):
        return self._interface

    def add_robot(self, robot, controlled=False):
        self.robots.append(robot)
        if controlled:
            if self.controlled_robot is not None:
                raise Exception("Only one robot can be controlled at a time")

            self.controlled_robot = robot

    def process_updates(self):
        print("Proc updates")
        for update in self.updates:
            if update.command == 'move':
                robot = self.robots[update.robot_id]

                next_x = robot.x + update.args[0] * math.cos(
                    math.radians(robot.bearing.angle))
                next_y = robot.y + update.args[0] * math.sin(
                    math.radians(robot.bearing.angle))
                robot.set_position(min(max(next_x, 0), 5740),
                                   min(max(next_y, 0), 5740))
            elif update.command == 'rotate':
                self.robots[update.robot_id].set_bearing(
                    (self.robots[update.robot_id].bearing.angle +
                     update.args[0]) % 360)
            elif update.command == 'power':
                self.robots[update.robot_id].set_power(update.args[0],
                                                       update.args[1])
            elif update.command == 'stop':
                self.running = False
            else:
                print("Unknown command {}".format(update.command))
        for robot in self.robots:
            robot.run_motors(1 / self.fps)

    def run(self):
        self.running = True
        self.figure = plt.figure()
        self.animation = animation.FuncAnimation(self.figure,
                                                 self.logic_thread,
                                                 interval=1 / self.fps)

    def logic_thread(self, frame):
        if frame > self.duration * self.fps or not self.running:
            self.animation.pause()
            self.animation = None
            self.running = False
        self.process_updates()
        self.render(frame)

    def render_robots(self):
        for robot in self.robots:
            self.points_to_display.append(
                Point(robot.x, robot.y, 'C2', Label(str(robot.id), '')))

            direction_x = robot.x + robot.camera.range / 2 * math.cos(
                math.radians(robot.bearing.angle))
            direction_y = robot.y + robot.camera.range / 2 * math.sin(
                math.radians(robot.bearing.angle))
            self.lines_to_display.append(
                Line(robot.x, robot.y, direction_x, direction_y, 'C2'))

    def render_points(self):
        for point in self.points_to_display:
            plt.scatter(point.x,
                        point.y,
                        color='C1' if point.color == '' else point.color)
            if point.label != '':
                plt.text(point.x,
                         point.y,
                         point.label.text,
                         horizontalalignment='center',
                         verticalalignment='center',
                         fontsize=5,
                         color='w'
                         if point.lable.color == '' else point.label.color)

    def render_lines(self):
        for line in self.lines_to_display:
            plt.plot([line.start.x, line.end.x], [line.start.y, line.end.y],
                     linestyle=':' if line.linestyle == '' else line.linestyle,
                     color='C1' if line.color == '' else line.color)

    def frame_setup(self):
        fig = self.process_background()  # renders or loads the background
        if self.frame != 0:
            ax = plt.gca()
            ax.set_xlim([0, 6000])
            ax.set_ylim([0, 6000])
            self.render_points()
            self.render_position()
            plt.plot([self.real_position[0], self.fov_points[0][0]],
                     [self.real_position[1], self.fov_points[1][0]],
                     color='C5',
                     linestyle='--')
            plt.plot([self.real_position[0], self.fov_points[0][1]],
                     [self.real_position[1], self.fov_points[1][1]],
                     color='C5',
                     linestyle='--')
            plt.plot([self.fov_points[0][0], self.fov_points[0][1]],
                     [self.fov_points[1][0], self.fov_points[1][1]],
                     color='C5',
                     linestyle='--')
            plt.scatter(self.calculated_pos[0],
                        self.calculated_pos[1],
                        color='C3')  # location
            x = []
            y = []

            for i in range(len(marker_list)):
                if marker_list[i][0] in self.seen_ids:
                    x.append(marker_list[i][1])
                    y.append(marker_list[i][2])
            plt.scatter(x, y, color='green')

        plot_end = time.time()
        # print(f"{self.frame}/{self.max_frame}: {round(plot_end-start, 3)}")

        if self.frame == 0:
            fig.savefig(fname="background.png",
                        dpi=500,
                        transparent=True,
                        bbox_inches='tight',
                        pad_inches=0)
        else:
            path = 'out/{}.png'.format(self.frame)
            self.async_plotter.save(fig, path)

        self.frame += 1

    def update(self, update):
        self.updates.append(update)

    def camera(self, robot):
        return SimulatorCamera(self, self.config['camera']['fov'],
                               self.config['camera']['range'],
                               self.config['markers_list'], robot)

    def render(self, frame):
        plt.clf()
        plt.rcParams["figure.figsize"] = (6, 6)
        plt.xlim(0, 5750)
        plt.ylim(0, 5750)
        plt.axis("off")
        if frame == 0:
            lines, points = self.map.render_static(self.figure)
            self.lines_to_display = lines
            self.points_to_display = points
            self.render_points()
            self.render_lines()
            self.map.set_static(self.figure)
            return
        else:
            self.map.load_static(self.figure)
            lines, points = self.map.render_dynamic(self.figure)
            self.lines_to_display += lines
            self.points_to_display += points
            self.render_robots()
            self.render_points()
            self.render_lines()
            self.show_frame()


class RobotController():

    def __init__(self, max_frame=100, camera_fov=62, camera_distance=4000):
        self.calculated_pos = [0, 0]
        self.calculated_points = []
        self.future_movement = []
        self.seen_markers = []  # Stores the output of camera.see()

    def move(self, amount, rot=0):
        speed = MAX_SPEED / FPS
        rotation_speed = ROT_SPEED / FPS
        x_left = amount
        rot_left = rot
        while x_left > 0 or rot_left > 0:
            mov_x = min(x_left, speed)
            mov_rot = min(rot_left, rotation_speed)
            self.future_movement.append([mov_x, mov_rot])
            x_left -= mov_x
            rot_left -= mov_rot
            x_left = max(0, x_left)
            rot_left = max(0, rot_left)

    def process_seen_markers(self):
        self.calculated_points = []
        if len(self.seen_markers) == 0:
            return
        #process each seen marker and output a point on the map it suggests the location of the camera is
        circles = []
        for marker in self.seen_markers:
            point = (marker[2] * math.cos(math.radians(90 - marker[0])))
            circles.append([marker[1], marker[2], point, marker[3]])

        valid_points = [[], []]
        for i in range(len(circles)):
            if i + 1 != len(circles):
                a = i + 1
            else:
                a = 0
            circle_one = circles[i]
            circle_two = circles[a]
            marker_one = [
                marker_list[circle_one[3]][1], marker_list[circle_one[3]][2]
            ]
            marker_two = [
                marker_list[circle_two[3]][1], marker_list[circle_two[3]][2]
            ]

            # circle (0: psi, 1: dist, 2: point, 3: marker id)
            points = get_intersections(marker_one[0], marker_one[1],
                                       circle_one[1], marker_two[0],
                                       marker_two[1], circle_two[1])
            for point in points:
                if 0 < point[0] < 5750 and 0 < point[1] < 5750:
                    valid_points[0].append(point[0])
                    valid_points[1].append(point[1])
                    self.calculated_points.append(
                        [point[0], point[1], circles[i][3]])
        if len(valid_points[0]) == 0:
            return
        avg_x = sum(valid_points[0]) / len(valid_points[0])
        avg_y = sum(valid_points[1]) / len(valid_points[1])
        self.calculated_pos = [avg_x, avg_y]
        # print the percentage diffrence between the real position and the calculated position

        print("x: {}% y: {}%".format(
            round(
                100 * (self.real_position[0] - avg_x) / self.real_position[0],
                1),
            round(
                100 * (self.real_position[1] - avg_y) / self.real_position[1],
                1)))

    def run(self):
        # TODO: Check our interface is setupt and the simulator is running
        # TODO: Now choose what we are going to do (eg send movement commands, calulate position etc)
        pass


if __name__ == "__main__":
    sim = Simulator(CONFIG)
    sim.run()
    plt.show()

# %%
