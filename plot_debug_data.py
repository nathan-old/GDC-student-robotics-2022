import json
from random import random
import matplotlib.pyplot as plt


class PlotDebugData():
    def __init__(self):
        self.debug_file = None
        self.start_zone = 0
        self.angle = 90
        self.debug_data = []
        self.coords = [0, 0]

    def set_route_file(self):
        self.debug_file = input("Path to debug file >")
        return self.debug_file

    def set_start_zone(self):
        self.start_zone = int(input('Which starting zone? (0,1,2,3)'))

    def set_start_angle(self):
        self.angle = int(
            input('What angle do you wish to start at? (Bearing form 0 is top)'))

    def debug_file_to_array(self):
        with open(self.debug_file, 'r') as route_data:
            data = route_data.readline()
            self.debug_data = json.loads(data)
            print(data)
            print(self.debug_data)

    def start_zone_to_coords(self):
        if self.start_zone == 0:
            self.coords = [0.5, 5.25]
        elif self.start_zone == 1:
            self.coords = [5.25, 5.25]
        elif self.start_zone == 2:
            self.coords = [5.25, 0.5]
        else:
            self.coords = [0.5, 0.5]

    def create_plot(self):
        # Assumes self.debug_data has been populated
        x = []
        y = []
        for i in self.debug_data:
            # Extract pos
            pos = i["calculated_position"]
            if not pos: # TODO: remove  this is just for testing
                pos = [[random() * 5.75, random() * 5.75], random() * 180, []]
            x.append(pos[0][0])
            y.append(pos[0][1])
        print(x, y)
        plt.plot(x, y)

        plt.xlim(0, 5.75)
        plt.ylim(0, 5.75)
        cans = [[0.0335, 2.875], [1.135, 2.875], [1.6, 1.6], [1.6, 4.15], [1.875, 2.475], [1.875, 3.275], [2.475, 1.875], [2.475, 3.875], [2.875, 0.0335], [2.875, 1.135], [
            2.875, 4.615], [2.875, 5.7165], [3.275, 1.875], [3.275, 3.875], [3.875, 3.275], [3.875, 2.475], [4.15, 1.6], [4.15, 4.15], [4.615, 2.875], [5.7165, 2.875]]
        x = []
        y = []
        for i in cans:
            x.append(i[0])
            y.append(i[1])
        plt.scatter(x, y, color='C1')

        plt.show()

pdd = PlotDebugData()
pdd.set_route_file()
pdd.set_start_angle()
pdd.set_start_zone()
pdd.debug_file_to_array()
pdd.create_plot()