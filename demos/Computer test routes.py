import matplotlib.pyplot as plt
import math


class RunRoute():
    def __init__(self):
        self.route_file = None
        self.start_zone = 0
        self.angle = 90
        self.route = []
        self.coords = [[0, 0]]

    def set_route_file(self):
        self.route_file = input("Path to .route file >")
        return self.route_file

    def set_bkg_file(self, plt):
        bk = input("Path to background file >")
        if bk != "":
            plt.imshow(plt.imread(bk), extent=[0, 5.75, 0, 5.75])

    def set_start_zone(self, value=4):
        if value == 4:
            self.start_zone = int(input('Which starting zone? (0,1,2,3)'))
        else:
            self.set_start_zone = value

    def set_start_angle(self):
        if int(self.start_zone) == 0:
            self.angle = 105
        elif int(self.start_zone) == 1:
            self.angle = 195
        elif int(self.start_zone) == 2:
            self.angle = 285
        elif int(self.start_zone) == 3:
            self.angle = 15
        # self.angle = int(input('What angle do you wish to start at? (Bearing form 0 is top)'))

    def route_file_to_array(self):
        with open(self.route_file, 'r') as route_data:
            route = route_data.readlines()
            for i in range(len(route)):
                if route[i] == '\n':
                    continue
                self.route.append(route[i].split(','))

    def start_zone_to_coords(self):
        if self.start_zone == 0:
            self.coords = [[0.5, 5.25]]
        elif self.start_zone == 1:
            self.coords = [[5.25, 5.25]]
        elif self.start_zone == 2:
            self.coords = [[5.25, 0.5]]
        else:
            self.coords = [[0.5, 0.5]]

    def follow_instructions(self):
        for i in self.route:
            print(i)
            if i[0] == 'forwards':
                print("forwards ", str(i[1]))
                x = math.sin(math.radians(self.angle))*float(i[1])
                y = math.cos(math.radians(self.angle))*float(i[1])
                new_x = ((self.coords[-1][0])+x)
                new_y = ((self.coords[-1][1])+y)
                self.coords.append([new_x, new_y])
            elif i[0] == "turn":
                print("turn {}".format(i[1]))
                bearing = self.angle
                bearing += float(i[1])
                if bearing > 360:
                    bearing -= 360
                elif bearing < 0:
                    bearing += 360
                self.angle = bearing
            elif i[0] == "sideways":
                bearing = self.angle
                bearing -= 90
                if bearing > 360:
                    bearing -= 360
                elif bearing < 0:
                    bearing += 360
                self.angle = bearing
                x = math.sin(math.radians(self.angle))*float(i[1])
                y = math.cos(math.radians(self.angle))*float(i[1])
                new_x = ((self.coords[-1][0])+x)
                new_y = ((self.coords[-1][1])+y)
                self.coords.append([new_x, new_y])
                bearing = self.angle
                bearing += 90
                if bearing > 360:
                    bearing -= 360
                elif bearing < 0:
                    bearing += 360
                self.angle = bearing
            elif i[0] == "bearing":
                self.angle += 150


runroute = RunRoute()
runroute.set_route_file()
runroute.set_bkg_file(plt)
runroute.set_start_zone(0)
runroute.set_start_angle()
runroute.route_file_to_array()
runroute.start_zone_to_coords()
runroute.follow_instructions()
x = []
y = []
for i in runroute.coords:
    x.append(i[0])
    y.append(i[1])
plt.plot(x, y)


runroute.start_zone = 1
runroute.set_start_angle()
runroute.start_zone_to_coords()
runroute.follow_instructions()
x = []
y = []
for i in runroute.coords:
    x.append(i[0])
    y.append(i[1])
plt.plot(x, y)


runroute.start_zone = 2
runroute.set_start_angle()
runroute.start_zone_to_coords()
runroute.follow_instructions()
x = []
y = []
for i in runroute.coords:
    x.append(i[0])
    y.append(i[1])
plt.plot(x, y)


runroute.start_zone = 3
runroute.set_start_angle()
runroute.start_zone_to_coords()
runroute.follow_instructions()
x = []
y = []
for i in runroute.coords:
    x.append(i[0])
    y.append(i[1])
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
