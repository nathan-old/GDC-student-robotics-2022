# %%
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import multiprocessing as mp
import time, matplotlib
import cv2, math, numpy as np

matplotlib.use('Agg')

central_box = [[2275, 2275], [2275, 3475], [3475, 3475], [3475, 2275],
               [2275, 2275]]
arena_border = [[0, 0], [0, 5750], [5750, 5750], [5750, 0], [0, 0]]
can_list = [[2875, 33.5], [33.5, 2875], [2875, 5716.5], [5716.5, 2875],
            [900, 900], [4850, 900], [4850, 4850], [900, 4850], [1135, 2875],
            [2875, 1135], [4615, 2875], [2875, 4615]]
point_zone = [[2500, 0], [3250, 0], [5750, 2500], [5750, 3250], [3250, 5750],
              [2500, 5750], [0, 3250], [0, 2500], [2500, 0]]
start_boxes = [[0, 1000], [1000, 1000], [1000, 0], [4750, 0], [4750, 1000],
               [5750, 1000], [5750, 4750], [4740, 4750], [4750, 5750],
               [1000, 5750], [1000, 4750], [0, 4750], [0, 1000]]

marker_list = [[0, 718.75, 5750], [1, 1437.5, 5750], [2, 2156.25, 5750],
               [3, 2875, 5750], [4, 3593.75, 5750], [5, 4312.5, 5750],
               [6, 5031.25, 5750], [7, 5750, 5031.25], [8, 5750, 4312.5],
               [9, 5750, 3593.75], [10, 5750, 2875], [11, 5750, 2156.25],
               [12, 5750, 1437.5], [13, 5750, 718.75], [14, 5031.25, 0],
               [15, 4312.5, 0], [16, 3593.75, 0], [17, 2875, 0],
               [18, 2156.25, 0], [19, 1437.5, 0], [20, 718.75, 0],
               [21, 0, 718.75], [22, 0, 1437.5], [23, 0, 2156.25],
               [24, 0, 2875], [25, 0, 3593.75], [26, 0, 4312.5],
               [27, 0, 5031.25]]

MAX_SPEED = 500  # Max speed (mm/s)
ROT_SPEED = 90  # degrees / s
FPS = 4
DURATION = 15  # secconds


# A function to check whether point P(x, y)
# lies inside the triangle formed by
# A(x1, y1), B(x2, y2) and C(x3, y3)
# Credit: https://www.geeksforgeeks.org/check-whether-a-given-point-lies-inside-a-triangle-or-not/
def isInside(x1, y1, x2, y2, x3, y3, x, y):

    def area(x1, y1, x2, y2, x3, y3):
        return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)

    # Calculate area of triangle ABC
    A = area(x1, y1, x2, y2, x3, y3)
    # Calculate area of triangle PBC
    A1 = area(x, y, x2, y2, x3, y3)
    # Calculate area of triangle PAC
    A2 = area(x1, y1, x, y, x3, y3)
    # Calculate area of triangle PAB
    A3 = area(x1, y1, x2, y2, x, y)
    # Check if sum of A1, A2 and A3
    # is same as A
    if (A == A1 + A2 + A3):
        return True
    else:
        return False


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


# Credit: https://gist.github.com/astrofrog/1453933
class AsyncPlotter():

    def __init__(self, processes=mp.cpu_count()):

        self.manager = mp.Manager()
        self.nc = self.manager.Value('i', 0)
        self.pids = []
        self.processes = processes

    def async_plotter(self, nc, fig, filename, processes):
        while nc.value >= processes:
            time.sleep(0.1)
        nc.value += 1
        fig.savefig(fname=filename,
                    dpi=500,
                    transparent=True,
                    bbox_inches='tight',
                    pad_inches=0)
        plt.close(fig)
        nc.value -= 1

    def save(self, fig, filename):
        p = mp.Process(target=self.async_plotter,
                       args=(self.nc, fig, filename, self.processes))
        p.start()
        self.pids.append(p)

    def join(self):
        for p in self.pids:
            p.join()


# calculates the position of the robot using the position of the marker
def calculate_pos(x_in, y_in, marker_number):
    robot_pos = []
    for i in range(len(marker_list)):
        if marker_list[i][0] == marker_number:
            marker_pos = [marker_list[i][1], marker_list[i][2]]
    if marker_number >= 0 and marker_number <= 6:
        x = marker_pos[0] - y_in
        y = marker_pos[1] - x_in
        robot_pos = [x, y]
    elif marker_number >= 7 and marker_number <= 13:
        x = marker_pos[0] - x_in
        y = marker_pos[1] + y_in
        robot_pos = [x, y]
    elif marker_number >= 14 and marker_number <= 20:
        x = marker_pos[0] + y_in
        y = marker_pos[1] + x_in
        robot_pos = [x, y]
    elif marker_number >= 19 and marker_number <= 27:
        x = marker_pos[0] + x_in
        y = marker_pos[1] + y_in
        robot_pos = [x, y]
    return (robot_pos)


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


# inputs from robot camera in degrees and mm
# list of markers seen by camera, in order theta, psi, r, marker number
seen_markers_ = [[85, 90, 1500, 11], [90, 90, 800, 8], [90, 90, 5750 / 2, 24],
                 [90, 90, 5750 / 2, 17], [90, 90, 1000, 23],
                 [90, 90 - 35.678, 1231.1, 22], [90, 90 + 35.678, 1231.1, 24]]

#seen_markers.append([-2.415157355926346, -11.116173340480376, 2086000, 0])


class Robot():

    def __init__(self, max_frame=5, camera_fov=60, camera_distance=4000):
        self.seen_ids = []
        self.seen_markers = []
        self.camera_fov = camera_fov
        self.camera_distance = camera_distance  # mm
        self.real_position = [5750 / 2, 5750 / 2]
        self.calculated_pos = [0, 0]
        self.frame = 0
        self.max_frame = max_frame
        self.calculated_points = []
        self.save_threads = []
        self.future_movement = []
        self.async_plotter = AsyncPlotter()
        self.rotation = 5
        self.fov_points = []

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

    def calc_move(self):
        if len(self.future_movement) == 0:
            return
        # get the first elemtn of self.future_movement and then remove it
        movement = self.future_movement.pop(0)
        self.rotation += movement[1]
        self.rotation = self.rotation % 360
        next_x = self.real_position[0] + movement[0] * math.cos(
            math.radians(self.rotation))
        next_y = self.real_position[1] + movement[0] * math.sin(
            math.radians(self.rotation))

        self.real_position[0] = next_x  #min(max(next_x, 0), 5740)
        self.real_position[1] = next_y  #min(max(next_y, 0), 5740)

    def calculate_seen(self):
        # TODO: emulate fov, figure out what markers we can see and pass this to calculate_seen_values
        self.seen_ids = []
        for marker in marker_list:
            id = marker[0]
            x = marker[1]
            y = marker[2]
            if point_in_triangle(
                (x, y), ((self.fov_points[0][0], self.fov_points[1][0]),
                         (self.real_position[0], self.real_position[1]),
                         (self.fov_points[0][1], self.fov_points[1][1]))):
                self.seen_ids.append(id)

        self.seen_markers = self.calculate_seen_values()

    def calculate_seen_values(self):
        # The values retruned from the "camera" in order theta, psi, r(distance), marker number
        points = []  #[[90, 90 - 35.678, 1231.1, 8],
        #            [90, 90, 1000, 9],
        #            [90, 90 + 35.678, 1231.1, 10]]
        for id in self.seen_ids:
            points.append([
                90,
                self.angle_to_point(marker_list[id][1], marker_list[id][2]),
                self.distance_to_point(marker_list[id][1], marker_list[id][2]),
                id
            ])

        self.seen_vals = points

        return points

    def process_seen_markers(self):
        self.calculated_points = []
        if len(self.seen_markers) == 0:
            return
        #process each seen marker and output a point on the map it suggests the location of the camera is
        circles = []
        rot = self.rotation
        factor = 180
        #if rot >= 0 and rot <= 90:
        #    factor  = 90
        #elif rot >= 90 and rot <= 180:
        #    factor = 180
        #elif rot >= 180 and rot <= 270:
        #    factor = 270
        #else:
        #    factor = 360
        #print(rot, factor)
        for marker in self.seen_markers:
            print(marker[3], marker[1])
            point = (marker[2] * math.cos(math.radians(factor - 90)))
            circles.append([marker[1], marker[2], point, marker[3]])
            plt.gca().add_patch(
                patches.Circle((marker[1], marker[2]),
                               point,
                               fill=False,
                               linestyle=':',
                               color='C1'))

        valid_points = [[], []]
        for i in range(len(circles)):
            if i + 1 != len(circles):
                a = i + 1
            else:
                a = 0
            points = get_intersections(circles[i][0], circles[i][1],
                                       circles[i][2], circles[a][0],
                                       circles[a][1], circles[a][2])
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
            100 * (self.real_position[0] - avg_x) / self.real_position[0],
            100 * (self.real_position[1] - avg_y) / self.real_position[1]))

    def calc_fov_points(self):
        x = self.real_position[0]
        y = self.real_position[1]
        endy = [
            y + self.camera_distance *
            math.sin(math.radians(self.rotation - self.camera_fov / 2)),
            y + self.camera_distance *
            math.sin(math.radians(self.rotation + self.camera_fov / 2))
        ]
        endx = [
            x + self.camera_distance *
            math.cos(math.radians(self.rotation - self.camera_fov / 2)),
            x + self.camera_distance *
            math.cos(math.radians(self.rotation + self.camera_fov / 2))
        ]
        self.fov_points = [endx, endy]

    def stich(self):
        img_array = []
        size = (0, 0)
        for frame_no in range(1, self.max_frame + 1):
            filename = "out/{}.png".format(frame_no)
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width, height)
            img_array.append(img)

        out = cv2.VideoWriter('project.mp4', cv2.VideoWriter_fourcc(*'mp4v'),
                              FPS, size)

        for i in range(len(img_array)):
            out.write(img_array[i])
        out.release()

    def render_position(self):
        plt.scatter(self.real_position[0], self.real_position[1], color='C2')

        plt.text(self.real_position[0],
                 self.real_position[1],
                 str(self.frame),
                 horizontalalignment='center',
                 verticalalignment='center',
                 fontsize=5,
                 color='w')

        direction_x = self.real_position[
            0] + self.camera_distance / 2 * math.cos(
                math.radians(self.rotation))
        direction_y = self.real_position[
            1] + self.camera_distance / 2 * math.sin(
                math.radians(self.rotation))
        plt.plot([self.real_position[0], direction_x],
                 [self.real_position[1], direction_y],
                 color='C1')

        # line going right from the robot
        plt.plot([
            self.real_position[0],
            self.real_position[0] + self.camera_distance / 2 *
            math.cos(math.radians(self.rotation - 90))
        ], [
            self.real_position[1],
            self.real_position[1] + self.camera_distance / 2 *
            math.sin(math.radians(self.rotation - 90))
        ],
                 color='C1',
                 linestyle='--')

    def render_points(self):
        for point in self.calculated_points:
            plt.scatter(point[0], point[1], color='C1')
            plt.text(point[0],
                     point[1],
                     str(point[2]),
                     horizontalalignment='center',
                     verticalalignment='center',
                     fontsize=5,
                     color='w')

            plt.plot([point[0], marker_list[point[2]][1]],
                     [point[1], marker_list[point[2]][2]],
                     linestyle=':',
                     color='C1')

    def process_background(self):
        fig = plt.figure()
        if self.frame == 0:
            plt.xlim(0, 5750)
            plt.ylim(0, 5750)

            x = []
            y = []
            for i in range(len(start_boxes)):
                x.append(start_boxes[i][0])
                y.append(start_boxes[i][1])
            plt.plot(x, y, 'b')

            x = []
            y = []
            for i in range(len(central_box)):
                x.append(central_box[i][0])
                y.append(central_box[i][1])
            plt.plot(x, y)
            x = []
            y = []
            for i in range(len(point_zone)):
                x.append(point_zone[i][0])
                y.append(point_zone[i][1])
            plt.plot(x, y, linestyle='--', color='r')
            x = []
            y = []
            for i in range(len(arena_border)):
                x.append(arena_border[i][0])
                y.append(arena_border[i][1])
            plt.plot(x, y, 'b')
            x = []
            y = []
            for i in range(len(marker_list)):
                x.append(marker_list[i][1])
                y.append(marker_list[i][2])
            plt.scatter(x, y, color='b')
            #plt.axvspan(0, 2000, alpha=0.7)
            #plt.axvspan(3750, 5750, alpha=0.7)
            #plt.axhspan(0, 2000, alpha=0.7)
            #plt.axhspan(3750, 5750, alpha=0.7)
            #plt.gca().add_patch(
            #    patches.Rectangle((2000, 2275), 1750, 1200, alpha=0.7))
            #plt.gca().add_patch(
            #    patches.Rectangle((2275, 2000), 1200, 1750, alpha=0.7))
            #plt.gca().add_patch(
            #    patches.Rectangle((2275, 2275), 1200, 1200, alpha=1,
            #                      color='b'))

            for i in range(len(x)):
                if i < 7:
                    plt.text(x[i],
                             y[i] - 40,
                             str(i),
                             horizontalalignment='center',
                             verticalalignment='center',
                             fontsize=5,
                             color='w')
                elif 6 < i < 14:
                    plt.text(x[i] - 60,
                             y[i],
                             str(i),
                             horizontalalignment='center',
                             verticalalignment='center',
                             fontsize=5,
                             color='w')
                elif 13 < i < 21:
                    plt.text(x[i],
                             y[i] + 40,
                             str(i),
                             horizontalalignment='center',
                             verticalalignment='center',
                             fontsize=5,
                             color='w')
                elif 20 < i < 28:
                    plt.text(x[i] + 60,
                             y[i],
                             str(i),
                             horizontalalignment='center',
                             verticalalignment='center',
                             fontsize=5,
                             color='w')
                else:
                    plt.text(x[i],
                             y[i],
                             str(i),
                             horizontalalignment='center',
                             verticalalignment='center',
                             fontsize=5,
                             color='w')

            plt.axis("off")
        else:
            # load the precreated background.png
            img = plt.imread("background.png")
            fig, ax = plt.subplots()
            ax.imshow(img, extent=[0, 5750, 0, 5750], aspect='auto', zorder=0)

        return fig

    def angle_to_point(self, x, y):
        robot_angle = self.rotation
        dx = x - self.real_position[0]
        dy = y - self.real_position[1]

        robot_angle -= 90
        if robot_angle < 0:
            robot_angle += 360
        robot_angle = 360 - robot_angle
        if robot_angle == 360:
            robot_angle = 0
        print("Leo: {}, Tobys: {}".format(self.rotation, robot_angle))
        marker_angle = None
        if dx > 0 and dy > 0:
            dx = abs(dx)
            dy = abs(dy)
            marker_angle = math.degrees(math.atan(dy / dx))
            marker_angle = 90 - marker_angle
        elif dx > 0 and dy < 0:
            dx = abs(dx)
            dy = abs(dy)
            marker_angle = math.degrees(math.atan(dy / dx))
            marker_angle = 90 + marker_angle
        elif dx < 0 and dy > 0:
            dx = abs(dx)
            dy = abs(dy)
            marker_angle = math.degrees(math.atan(dy / dx))
            marker_angle = 270 - marker_angle
        elif dx < 0 and dy < 0:
            dx = abs(dx)
            dy = abs(dy)
            marker_angle = math.degrees(math.atan(dy / dx))
            marker_angle = 270 + marker_angle
        elif dx == 0 and dy > 0:
            marker_angle = 0
        elif dx == 0 and dy < 0:
            marker_angle = 270
        elif dy == 0 and dx > 0:
            marker_angle = 90
        elif dy == 0 and dx < 0:
            marker_angle = 180

        if marker_angle == 0 and robot_angle > 180:
            marker_angle = 360
        phi = (-1 * (marker_angle - robot_angle)) + 90

        return phi

    def distance_to_point(self, x, y):
        dx = x - self.real_position[0]
        dy = y - self.real_position[1]
        return math.sqrt(dx**2 + dy**2)

    def render_frame(self):

        start = time.time()

        plt.clf()

        plt.rcParams["figure.figsize"] = (6, 6)
        fig = self.process_background()  # renders or loads the background
        if self.frame != 0:
            ax = plt.gca()
            ax.set_xlim([0, 6000])
            ax.set_ylim([0, 6000])
            self.render_points()
            self.render_position()
            plt.plot([self.real_position[0], self.fov_points[0][0]],
                     [self.real_position[1], self.fov_points[1][0]],
                     color='C5')
            plt.plot([self.real_position[0], self.fov_points[0][1]],
                     [self.real_position[1], self.fov_points[1][1]],
                     color='C5')
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

            for point in self.seen_vals:
                # draw a line from the robot at angle and distance
                direction_x = self.real_position[0] + point[2] * math.cos(
                    math.radians(point[1]))
                direction_y = self.real_position[0] + point[2] * math.sin(
                    math.radians(point[1]))
                plt.plot([self.real_position[0], direction_x],
                         [self.real_position[1], direction_y],
                         color='blue')

        plot_end = time.time()
        print(f"{self.frame}/{self.max_frame}: {round(plot_end-start, 3)}")

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

    def run(self):
        self.calc_fov_points()
        while True:
            if self.frame == self.max_frame + 1:
                self.async_plotter.join()
                self.stich()
                break
            if len(self.future_movement) == 0:
                self.move(0, 10)

            self.calc_move()
            self.calc_fov_points()
            self.calculate_seen()
            self.process_seen_markers()
            self.render_frame()
            time.sleep(0.05)


import os, shutil
if os.path.exists("out"):
    shutil.rmtree("out")
os.makedirs("out")
time.sleep(2)
r = Robot(DURATION * FPS)
r.run()

# %%
