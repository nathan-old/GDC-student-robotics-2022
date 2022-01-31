import math
import random
from primatives import *
import pygame
from pygame.locals import *

from collider import Rectangle, World


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

    @property
    def x(self):
        return self._position.x

    @property
    def y(self):
        return self._position.y

    @property
    def id(self):
        return self._id


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
FPS = 30
DURATION = 5  # secconds#

M0 = 0
M1 = 0
M2 = 0

class Renderer():
    def __init__(self, res_x, res_y):
        self.display = None
        self.res_x = res_x
        self.res_y = res_y
        self.clock = None
        self.running = False
        self.frame = 0
        self.points = []
        self.lines = []
        self.font = None

    def normalise_arena_pos(self, pos):
        return Vector2D(pos.x/5750, pos.y/5750)

    def position_to_screenspace(self, pos):
        return Vector2D((pos.x * self.res_x) + self.res_x * 0.05, (self.res_y + self.res_y * 0.05) - (pos.y * self.res_y) )

    def start(self):
        pygame.init()
        self.display = pygame.display.set_mode((self.res_x + self.res_x * 0.1, self.res_y  + self.res_y * 0.1), RESIZABLE)
        pygame.display.set_caption("Robot Arena")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, int(0.03 * self.res_x))

    def rect(self, pos, w, h, color):
        pygame.draw.rect(self.display, color, [pos.x, pos.y, w, h])

    def line(self, line):
        # print("Line: start ({}, {}), end ({}, {})".format(line.start.x, line.start.y, line.end.x, line.end.y))
        pygame.draw.line(self.display, line.color, (line.start.x,
                         line.start.y), (line.end.x, line.end.y))

    def circle(self, point):
        if point.display:
            pygame.draw.circle(self.display, point.color, (point.x,
                           point.y), int(0.01 * self.res_x), int(0.01 * self.res_y))

    def text(self, text, position, color=pygame.Color("white")):
        self.display.blit(self.font.render(text, True, color),
                         (position.x, position.y))

    def loop(self, logic_callback):
        global M0, M1, M2
        if self.display is None:
            raise Exception("Renderer not started")
        self.running = True

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == VIDEORESIZE:
                    self.res_x = event.w
                    self.res_y = event.h
                    self.start()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_LEFT:
                        M2 -= 0.1
                        M2 = max(min(M2, 1), -1)
                    if event.key == pygame.K_RIGHT:
                        M2 += 0.1
                        M2 = max(min(M2, 1), -1)
                    if event.key == pygame.K_UP:
                        M1 += 0.1
                        M1 = max(min(M1, 1), -1)
                        M0 = M1
                    if event.key == pygame.K_DOWN:
                        M1 -= 0.15
                        M1 = max(min(M1, 1), -1)
                        M0 = M1
            logic_callback(self.frame, 1 / FPS)

            self.display.fill((0, 0, 0))
            for point in self.points:
                point_pos = self.position_to_screenspace(
                    self.normalise_arena_pos(point.position))
                point._x = point_pos.x
                point._y = point_pos.y
                self.circle(point)
                if point.label is not None:
                    label = point.label
                    label_pos = point.position
                    if label.offset.x != 0 or label.offset.x != 0:
                        label_pos._x += label.offset.x
                        label_pos._y += label.offset.y
                    self.text(label.text, label_pos, label.color)

            for line in self.lines:
                line._start = self.position_to_screenspace(
                    self.normalise_arena_pos(line.start))
                line._end = self.position_to_screenspace(
                    self.normalise_arena_pos(line.end))
                self.line(line)

            pygame.display.flip()
            self.frame += 1
            self.clock.tick(FPS)

        pygame.quit()
        quit()

    def set(self, lines, points):
        self.lines = lines
        self.points = points

    def add_point(self, point):
        self.points.append(point)
    
    def add_line(self, line):
        self.lines.append(line)


class Map():

    def __init__(self, central_box, arena_border, can_list, point_zone,
                 start_boxes, marker_list):
        self.central_box = central_box
        self.arena_border = arena_border
        self.can_list = can_list
        self.point_zone = point_zone
        self.start_boxes = start_boxes
        self.marker_list = marker_list
        self.ax = None
        self.static = None
        self.central_box_bounding_box = None

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
            lines.append(
                Line(self.start_boxes[i], self.start_boxes[i + 1], pygame.Color('blue')))

        # Draw central raised platform
        for i in range(len(self.central_box) - 1):
            lines.append(Line(self.central_box[i], self.central_box[i + 1],
                              pygame.Color('grey')))

        # Draw point zones
        for i in range(len(self.point_zone)-1):
            lines.append(
                Line(self.point_zone[i], self.point_zone[i+1], pygame.Color('red'), '--'))

        # Draw arena border
        for i in range(len(self.arena_border)-1):
            lines.append(
                Line(self.arena_border[i], self.arena_border[i + 1], pygame.Color('blue')))

        return (lines, points)

    # Renders the dynamic elements (cans, markers)
    def render_dynamic(self):
        # Draw markers
        lines = [] #self.central_box_bounding_box.as_lines()
        points = []
        for marker in self.marker_list:
            points.append(
                Point(marker.x, marker.y, (0, 255, 0) if marker.can_see else (255, 0, 0), Label(str(marker.id), (255, 255, 255))))

        for i in range(len(points)):
            if i < 7:
                points[i]._label = Label(
                    str(i), (255, 255, 255), Vector2D(0, -40))
            elif 6 < i < 14:
                points[i]._label = Label(
                    str(i), (255, 255, 255), Vector2D(0, -60))
            elif 13 < i < 21:
                points[i]._label = Label(
                    str(i), (255, 255, 255), Vector2D(0, +40))
            elif 20 < i < 28:
                points[i]._label = Label(
                    str(i), (255, 255, 255), Vector2D(0, +60))
            else:
                points[i]._label = Label(str(i), (255, 255, 255))
            return (lines, points)

        # Draw cans
        for i in range(len(can_list)):
            points.append(
                Point(can_list[i].x, can_list[i].y, pygame.Color('black'), Label(str(i), (255, 255, 255))))


    def populate_collider(self, world):
        #self.central_box_bounding_box = Rectangle(self.central_box[0].x, self.central_box[0].y, self.central_box[2].x - self.central_box[0].x, self.central_box[2].y - self.central_box[0].y)
        #world.add(self.central_box_bounding_box)
        # TODO: add walls and cans
        pass

        


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
        try:
            a = (r0**2 - r1**2 + d**2) / (2 * d)
            h = math.sqrt(r0**2 - a**2)
            x2 = x0 + a * (x1 - x0) / d
            y2 = y0 + a * (y1 - y0) / d
            x3 = x2 + h * (y1 - y0) / d
            y3 = y2 - h * (x1 - x0) / d
            x4 = x2 - h * (y1 - y0) / d
            y4 = y2 + h * (x1 - x0) / d
            return ([[x3, y3], [x4, y4]])
        except:
            return [[-4, -4], [-4, -4]]


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

    def __init__(self, sim, fov, range, marker_list, id):
        self.sim = sim
        self.fov = fov
        self.range = range
        self.robot = self.sim.get_robot(id)
        self.marker_list = marker_list
        self.can_see = []
        self.posed_markers = []

    @property
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
            self.sim.map.marker_list[marker.id]._can_see = False
            if point_in_triangle(
                (marker.x, marker.y), ((self.fov_points[0][0], self.fov_points[1][0]),
                                       (self.robot.x, self.robot.y),
                                       (self.fov_points[0][1], self.fov_points[1][1]))):
                self.can_see.append(marker.id)
                self.sim.map.marker_list[marker.id]._can_see = True

    def pose_estimation(self):
        # The values retruned from the "camera" in order theta, psi, r(distance), marker number
        self.posed_markers = []
        for id in self.can_see:
            self.posed_markers.append([
                90,
                self.angle_to_point(self.marker_list[id].x,
                                    self.marker_list[id].y),
                self.distance_to_point(self.marker_list[id].x,
                                       self.marker_list[id].y), id
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

    def check(self):
        salt = random.randint(0, 256)
        cmd = SimulatorCommand(None, 'check', [salt])
        return self.parent.update(cmd) == salt

    def camera(self, id):
        return self.parent.get_camera(id)

    def try_move(self, box, pos):
        return self.parent.try_move(box, pos)
    
    def add_point(self, point):
        self.parent.renderer.add_point(point)

    def add_line(self, line):
        self.parent.renderer.add_line(line)


class RobotInterface():

    def __init__(self, position, bearing, interface, id=0):
        self._pos = position
        self._bearing = bearing
        self._id = id
        self._interface = interface
        self.motors = [0, 0, 0]  # Power levels for each motor
        self.wheel_arm_lengths = [0, 0, 0]  # Length of each wheel arm
        self.wheel_radius = 50  # Radius of the wheels in mm
        self.max_rpm = 90  # Maximum RPM of each motor
        self.wheels_distance = 250 # Distance from center of robot to each wheel in mm
        # Choose random color for robot
        r = random.random()
        b = random.random()
        g = random.random()
        self.bounding_box = []
        for wheel in self.wheel_positions():
            self.bounding_box.append(
                Rectangle(wheel.x - self.wheel_radius, wheel.y - self.wheel_radius, 2 * self.wheel_radius, 2 * self.wheel_radius))

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

    def wheel_positions(self,x=0,y=0):
        x = x if x != 0 else self.x
        y = y if y != 0 else self.y
        return [Vector2D(x-self.wheels_distance*math.cos(math.radians(self.bearing.angle)), y-self.wheels_distance*math.sin(math.radians(self.bearing.angle))),
                Vector2D(x+self.wheels_distance*math.cos(math.radians(self.bearing.angle + 30)), y+self.wheels_distance*math.sin(math.radians(self.bearing.angle + 30))),
                Vector2D(x+self.wheels_distance*math.cos(math.radians(self.bearing.angle - 30)), y+self.wheels_distance*math.sin(math.radians(self.bearing.angle - 30))),
            ]

    @property
    def interface(self):
        return self._interface

    @property
    def camera(self):
        return self._interface.camera(self._id)

    def set_position(self, x, y):
        # calculate the position of the wheels at new location
        #wheels = self.wheel_positions(x, y)
        #wheel_positions = []
        #i = 0
        #for wheel in wheels:
        #    wheel_pos = self._interface.try_move(self.bounding_box[i], Vector2D(wheel.x, wheel.y))
        #    wheel_positions.append(wheel_pos)
        #    self.bounding_box[i].pos = wheel_pos.x, wheel_pos.y
        #    i += 1
        
        # find the center of the wheel_positions
        #x_center = (self.bounding_box[0].pos.x + self.bounding_box[1].pos.x + self.bounding_box[2].pos.x) / 3
        #y_center = (self.bounding_box[0].pos.y + self.bounding_box[1].pos.y + self.bounding_box[2].pos.y) / 3
        #print(100 * ((x_center - x) / x))
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
    def wheel_velocites(self):
        velocities = []
        for i in range(len(self.motors)):
            rpm = self.motors[i] * self.max_rpm
            velocities.append((rpm * (2 * math.pi * self.wheel_radius)) / 60)
        return velocities # mm/s

    def run_motors(self, time):
        # TODO: Calculate the new x,y position and rotation using: Current motor speeds, motor/wheel positon info, current position and rotation and time (which is in secconds)
        # This is probably done by creating a force vector, multiplying it by time applying it to the robot
        wheel_velocities = self.wheel_velocites()
        print("Wheel velocities: {}".format(wheel_velocities))
        x = (math.sqrt(3)/3) * ((wheel_velocities[0] + wheel_velocities[1]) * time)
        y = (1/3) * ((wheel_velocities[0] + wheel_velocities[1]) * time) 
        if x != 0:
            x -= (2/3 * wheel_velocities[2] * time)
        front_angular = 0
        if wheel_velocities[0] > wheel_velocities[1]:
            front_angular = wheel_velocities[0] - wheel_velocities[1]
        elif wheel_velocities[0] < wheel_velocities[1]:
            front_angular = wheel_velocities[1] - wheel_velocities[0]

        angular =  (1/(3 * self.wheels_distance)) * ((wheel_velocities[2] + front_angular) * time)
        print("x: {} y: {}, w: {}".format(x, y, math.degrees(angular)))
        self._bearing._angle += math.degrees(angular)
        self._bearing._angle = round(self._bearing._angle % 360, 1)
        next_x = self.x + x * math.cos(
            math.radians(self.bearing.angle))
        next_y = self.y + y * math.sin(
            math.radians(self.bearing.angle))
        self.set_position(min(max(next_x, 0), 5740),
                        min(max(next_y, 0), 5740))


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
        self.map = Map(central_box, arena_border, can_list,
                       point_zone, start_boxes, marker_list)
        self.world = World()
        self.frame = 0
        self.renderer = None
        self.running = False

    @property
    def interface(self):
        return self._interface

    def try_move(self, box, position):
        # Try to move bounding box to position, returns a Vector2D of the final position
        # if the move is possible this will be the same as position
        # if the move is not possible this will be the closest possible position
        def filter(t,o):
            return "slide"
        return self.world.check_move(box, position, filter)[0]

    def add_robot(self, robot, controlled=False):
        self.robots.append(robot)
        for b in robot.bounding_box:
            self.world.add(b)
        
        if controlled:
            if self.controlled_robot is not None:
                raise Exception("Only one robot can be controlled at a time")

            self.controlled_robot = robot

    def get_robot(self, id):
        return self.robots[id]

    def process_updates(self):
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
                print("Unknown sim command {}".format(update.command))
        for robot in self.robots:
            robot.run_motors(1 / self.fps)

    def run(self):
        self.map.populate_collider(self.world)
        self.running = True
        self.renderer = Renderer(800, 800)
        self.renderer.start()
        self.renderer.loop(self.logic_thread)

    def logic_thread(self, i, t):
        self.renderer.set([],[])
        if self.frame > self.duration * self.fps or not self.running:
            print("exit case")
            self.animation = None
            self.running = False

        for robot in self.robots:
            robot.tick(t)
        self.process_updates()

        print("Frame {}/{}".format(self.frame, self.fps * self.duration))

        lines_static, points_static = self.map.render_static()
        lines, points = self.map.render_dynamic()
        print("Rendered dynamic map")
        robot_lines, robot_points = self.render_robots()
        print("Rendered {} robots".format(len(self.robots)))
        for line in lines_static + lines + robot_lines:
            self.renderer.add_line(line)
        for point in points_static + points + robot_points:
            self.renderer.add_point(point)
        self.frame = i

    def render_robots(self):
        points = []
        lines = []
        for robot in self.robots:
            points += [
                Point(robot.x-robot.wheels_distance*math.cos(math.radians(robot.bearing.angle)), robot.y-robot.wheels_distance*math.sin(math.radians(robot.bearing.angle)), (int(((robot.motors[2] +1) / 2) * 255), int(((robot.motors[2] + 1) / 2) * 255), 0), Label("M3", pygame.Color('white'))),
                Point(robot.x+robot.wheels_distance*math.cos(math.radians(robot.bearing.angle + 30)), robot.y+robot.wheels_distance*math.sin(math.radians(robot.bearing.angle + 30)), (int(((robot.motors[0] + 1) / 2) * 255), int(((robot.motors[0] + 1) / 2) * 255), 0), Label("M0", pygame.Color('white'))),
                Point(robot.x+robot.wheels_distance*math.cos(math.radians(robot.bearing.angle - 30)), robot.y+robot.wheels_distance*math.sin(math.radians(robot.bearing.angle - 30)), (int(((robot.motors[1] + 1) / 2) * 255), int(((robot.motors[1] + 1) / 2) * 255), 0), Label("M1", pygame.Color('white'))),
            ]

            direction_x = robot.x + robot.camera.range / 2 * math.cos(
                math.radians(robot.bearing.angle))
            direction_y = robot.y + robot.camera.range / 2 * math.sin(
                math.radians(robot.bearing.angle))
            lines.append(
                Line(Vector2D(robot.x, robot.y), Vector2D(direction_x, direction_y), pygame.Color('green')))
            # FOV points
            lines += [
                Line(robot.position, Vector2D(
                    robot.camera.fov_points[0][0], robot.camera.fov_points[1][0]), pygame.Color('orange')),
                Line(robot.position, Vector2D(
                    robot.camera.fov_points[0][1], robot.camera.fov_points[1][1]),  pygame.Color('orange')),
                Line(Vector2D(
                    robot.camera.fov_points[0][0], robot.camera.fov_points[1][0]), Vector2D(
                    robot.camera.fov_points[0][1], robot.camera.fov_points[1][1]),  pygame.Color('orange'))
            ] 
            #for box in robot.bounding_box:
            #    lines += box.as_lines()
            points.append(Point(direction_x, direction_y, pygame.Color('red'), Label("{}".format(robot.bearing.angle), pygame.Color('white')), False))
        return (lines, points)

    def update(self, update):
        self.updates.append(update)

    def get_camera(self, robot):
        return SimulatorCamera(self, self.config['camera']['fov'],
                               self.config['camera']['range'],
                               self.config['markers_list'], robot)


class RobotController(RobotInterface):

    def __init__(self, start_pos, start_bearing, interface, id):
        super().__init__(start_pos, start_bearing, interface, id)
        self.calculated_pos = [0, 0]
        self.calculated_points = []
        self.future_movement = []
        self.seen_markers = []  # Stores the output of camera.see()
        self.last_points = []
        self.last_point_every = 15
        self.last_points_counter = 0
        self.last_point_max = 20

    def move(self, distance):
        pass

    def process_seen_markers(self):
        self.calculated_points = []
        if len(self.seen_markers) == 0:
            return
        # process each seen marker and output a point on the map it suggests the location of the camera is
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
                marker_list[circle_one[3]].x, marker_list[circle_one[3]].y
            ]
            marker_two = [
                marker_list[circle_two[3]].x, marker_list[circle_two[3]].y
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
        x_diff = round(
                100 * (self.position.x - avg_x) / self.position.x,
                1)
        y_diff = round(
                100 * (self.position.y - avg_y) / self.position.y,
                1)
        if x_diff != 0 or y_diff != 0:
            print("x: {}% y: {}%".format(x_diff, y_diff))

    def tick(self, t):
        # Check our interface is setupt and the simulator is running
        if self.interface is None:
            raise Exception("Interface not set")
        # TODO: Now choose what we are going to do (eg send movement commands, calulate position etc)
        self.seen_markers=self.camera.see()
        self.process_seen_markers()
        self.interface.add_point(Point(self.calculated_pos[0], self.calculated_pos[1], pygame.Color('red'), Label(str(self.id), pygame.Color('white'))))
        #for i in range(len(self.last_points)):
        #    print(self.last_points[i].position.x, self.last_points[i].position.y)
        #    self.interface.add_point(self.last_points[i])
            #self.interface.add_line(Line(self.last_points[i].position, self.last_points[i+1].position, pygame.Color('red')))
            
        #self.last_points_counter += 1
        #if self.last_points_counter > self.last_point_every:
        #    self.last_points_counter = 0
        #    self.last_points.append(Point(int(self.calculated_pos[0]), int(self.calculated_pos[1]), pygame.Color('grey'), Label(str(len(self.last_points)), pygame.Color('white')), False))
        #    if len(self.last_points) > self.last_point_max:
        #        self.last_points.pop(0)
        self.motors[2] = M2
        self.motors[0] = M0
        self.motors[1] = M1

        pass


if __name__ == "__main__":
    sim=Simulator(CONFIG)
    sim.add_robot(RobotController(Vector2D(500, 500),
                  Bearing(50), sim.interface, 0))
    sim.run()
