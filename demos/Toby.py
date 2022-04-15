import matplotlib.pyplot as plt
import easygui, math, os


class RunRoute():
    def __init__(self):
        self.route_file = None
        self.start_zone = 0
        self.angle = 90
        self.route = []
        self.coords = [[0,0]]
    def set_route_file(self):
        self.route_file = easygui.fileopenbox(msg='Select a route file', title='Select a file', default='*.txt')
        return self.route_file
    def set_start_zone(self):
        self.start_zone = int(input('Which starting zone? (0,1,2,3)'))
    def set_start_angle(self):
        self.angle = int(input('What angle do you wish to start at? (Bearing form 0 is top)'))
    def route_file_to_array(self):
        with open(self.route_file,'r') as route_data:
            self.route = route_data.readlines()
            for i in range(len(self.route)):
                self.route[i] = self.route[i].split(', ')
        route_data.close()
    def start_zone_to_coords(self):
        if self.start_zone == 0:
            self.coords = [[0.5,5.25]]
        elif self.start_zone == 1:
            self.coords = [[5.25,5.25]]
        elif self.start_zone == 2:
            self.coords = [[5.25,0.5]]
        else:
            self.coords = [[0.5,0.5]]

    def follow_instructions(self):
        for i in self.route:
            if i[0] == 'forwards':
                x = math.sin(math.radians(self.angle))*float(i[1])
                y = math.cos(math.radians(self.angle))*float(i[1])
                new_x = ((self.coords[-1][0])+x)
                new_y = ((self.coords[-1][1])+y)
                self.coords.append([new_x,new_y])
            elif i[0] == "turn":
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
                self.coords.append([new_x,new_y])
                bearing = self.angle
                bearing += 90
                if bearing > 360:
                    bearing -= 360
                elif bearing < 0:
                    bearing += 360
                self.angle = bearing

runroute = RunRoute()
runroute.set_route_file()
runroute.set_start_zone()
runroute.set_start_angle()
runroute.route_file_to_array()
runroute.start_zone_to_coords()
runroute.follow_instructions()
print(runroute.coords)
x = []
y = []
for i in runroute.coords:
    x.append(i[0])
    y.append(i[1])
plt.xlim(0, 5.75)
plt.ylim(0, 5.75)
plt.imshow(plt.imread(os.path.join('background.png')), extent=[0, 5.750, 0, 5.750])
plt.plot(x,y)
plt.show()