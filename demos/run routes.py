import matplotlib.pyplot as plt
import easygui, math


class RunRoute():
    def __init__(self):
        self.route_file = None
        self.start_zone = 0
        self.angle = 90
        self.route = []
        self.coords = [0,0]
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
            self.coords = [0.5,5.25]
        elif self.start_zone == 1:
            self.coords = [5.25,5.25]
        elif self.start_zone == 2:
            self.coords = [5.25,0.5]
        else:
            self.coords = [0.5,0.5]

    def follow_instructions(self):
        for i in range(self.route):
            if i[0] == 'forwards':
                x = math.sin(math.radians(self.angle))*i[1]
                y = math.cos(math.radians(self.angle))*i[1]
                self.coords.append([self.coords[len(self.coords)][0]+x,self.coords[len(self.coords)][1]+y])
