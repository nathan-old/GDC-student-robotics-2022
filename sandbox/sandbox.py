# %%
from math import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import multiprocessing as mp
import time, matplotlib
import cv2
import glob

matplotlib.use('Agg')

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


# calculates the distance of the marker from the robot
def calculate_distance(theta, psi, r):
    h = (r * cos(radians(90 - theta)))
    z = (r * cos(radians(theta)))
    x = (h * cos(radians(90 - psi)))
    y = (h * sin(radians(90 - psi)))
    print("x:", x, end=', ')
    print("y:", y, end=', ')
    print("z:", z)
    return (x, y)


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


# inputs from robot camera in degrees and mm
# list of markers seen by camera, in order theta, psi, r, marker number
seen_markers_ = [[85, 90, 1500, 11], [90, 90, 800, 8], [90, 90, 5750 / 2, 24],
                 [90, 90, 5750 / 2, 17], [90, 90, 1000, 23],
                 [90, 90 - 35.678, 1231.1, 22], [90, 90 + 35.678, 1231.1, 24]]

#seen_markers.append([-2.415157355926346, -11.116173340480376, 2086000, 0])


class Robot():

    def __init__(self,
                 background_mode=True,
                 max_frame=5,
                 camera_fov=60,
                 camera_distance=3000):
        self.seen_markers = []
        self.camera_fov = camera_fov
        self.camera_distance = camera_distance  # mm
        self.real_position = [0, 0]
        self.calculated_pos = [0, 0]
        self.frame = 0
        self.max_frame = max_frame
        self.background_mode = background_mode
        self.calculated_points = []
        self.save_threads = []
        self.async_plotter = AsyncPlotter()

    def move(self, new_coords):
        self.real_position[0] += new_coords[0]
        self.real_position[1] += new_coords[1]
        self.calculate_seen()
        self.render_frame()

    def calculate_seen(self):
        return self.calculate_seen_values()

    def calculate_seen_values(self):
        return [[85, 90, 1500, 11], [90, 90, 800, 8], [90, 90, 5750 / 2, 24],
                [90, 90, 5750 / 2, 17], [90, 90, 1000, 23],
                [90, 90 - 35.678, 1231.1, 22], [90, 90 + 35.678, 1231.1, 24]]

    def process_seen_markers(self):
        for marker in self.seen_markers:
            (x, y) = calculate_distance(marker[0], marker[1], marker[2])
            point = calculate_pos(x, y, marker[3])
            point.append(marker[3])
            self.calculated_points.append(point)
        # TODO: calc standard deviation and remove outliers

    def run(self):
        if self.background_mode:
            self.render_frame()
            return
        while True:
            if self.frame == self.max_frame + 1:
                self.async_plotter.join()
                self.stich()
                break
            self.move([100, 100])
            time.sleep(0.05)

    def stich(self):
        FPS = 2
        img_array = []
        size = (0, 0)
        for frame_no in range(1, self.max_frame + 1):
            filename = "out/{}.png".format(frame_no)
            img = cv2.imread(filename)
            height, width, layers = img.shape
            size = (width, height)
            img_array.append(img)

        out = cv2.VideoWriter('project.avi', cv2.VideoWriter_fourcc(*'DIVX'),
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

    def render_points(self):
        for point in self.calculated_points:
            plt.scatter(point[0], point[1], color='C1')
            plt.text(point[0],
                     point[1],
                     str(point[3]),
                     horizontalalignment='center',
                     verticalalignment='center',
                     fontsize=5,
                     color='w')

            plt.plot([point[0], marker_list[point[3]][1]],
                     [point[1], marker_list[point[3]][2]],
                     linestyle=':',
                     color='C1')

    def process_background(self):
        fig = plt.figure()
        if self.frame == 0:

            central_box = [[2275, 2275], [2275, 3475], [3475, 3475],
                           [3475, 2275], [2275, 2275]]
            arena_border = [[0, 0], [0, 5750], [5750, 5750], [5750, 0], [0, 0]]
            can_list = [[2875, 33.5], [33.5, 2875], [2875, 5716.5],
                        [5716.5, 2875], [900, 900], [4850, 900], [4850, 4850],
                        [900, 4850], [1135, 2875], [2875, 1135], [4615, 2875],
                        [2875, 4615]]
            point_zone = [[2500, 0], [3250, 0], [5750, 2500], [5750, 3250],
                          [3250, 5750], [2500, 5750], [0, 3250], [0, 2500],
                          [2500, 0]]
            start_boxes = [[0, 1000], [1000, 1000], [1000, 0], [4750, 0],
                           [4750, 1000], [5750, 1000], [5750, 4750],
                           [4740, 4750], [4750, 5750], [1000, 5750],
                           [1000, 4750], [0, 4750], [0, 1000]]
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
            ax.imshow(img, extent=[0, 6000, 0, 6000], aspect='auto', zorder=0)

        return fig

    def render_frame(self):

        start = time.time()

        plt.clf()

        plt.rcParams["figure.figsize"] = (6, 6)
        fig = self.process_background()  # renders or loads the background
        if self.frame != 0:
            self.render_points()
            self.render_position()
        plot_end = time.time()
        print(f"Plotting time = {plot_end-start}")

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


import os, shutil
if os.path.exists("out"):
    shutil.rmtree("out")
os.makedirs("out")
time.sleep(2)
r = Robot(False, 5)
r.run()

# %%
