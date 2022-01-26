from math import *

# calculates the distance of the marker from the robot
def calculate_distance(theta,psi,r):
    h = (r*sin((90-theta))/180*pi)/sin(90/180*pi)
    z = (r*sin(theta/180*pi))/sin(90/180*pi)
    x = (z*sin((90-psi)/180*pi))/sin(90/180*pi)
    y = (z*sin(psi/180*pi))/sin(90/180*pi)
    print("x:",x) #not the x coordinate
    print("y:",y) #not the y coordinate
    print("z:",z) #distance from marker on xy plane
    print("h:",h) #height of marker off the ground
    return calculate_pos(x, y, z, marker_list, marker_number)

# calculates the position of the robot using the position of the marker
def calculate_pos(x,y,z,marker_list,marker_number):
    robot_pos = []
    for i in marker_list:
            if i[0] == marker_number:
                marker_pos = [i[1],i[2]]

    if marker_number > -1 and marker_number < 7:
        x = marker_pos[0]-x
        y = marker_pos[1]-y
        robot_pos = [x,y]
    elif marker_number > 6 and marker_number < 14:
        x = marker_pos[0]-y
        y = marker_pos[1]+x
        robot_pos = [x,y]
    elif marker_number > 13 and marker_number < 21:
        x = marker_pos[0]+x
        y = marker_pos[1]+y
        robot_pos = [x,y]
    elif marker_number > 20 and marker_number < 28:
        x = marker_pos[0]+y
        y = marker_pos[1]-x
        robot_pos = [x,y]
    else:
        print ("Invalid\ marker number")
    print(robot_pos)

# list of marker positions
marker_list = [[0,718.75,5750],[1,1437.5,5750],[2,2156.25,5750],[3,2875,5750],[4,3593.75,5750],[5,4312.5,5750],[6,5031.25,5750],[7,5750,5031.25],[8,5750,4312.5],[9,5750,3593.75],[10,5750,2875],[11,5750,2156.25],[12,5750,1437.5],[13,5750,718.75],[14,5031.25,0],[15,4312.5,0],[16,3593.75,0],[17,2875,0],[18,2156.25,0],[19,1437.5,0],[20,718.75,0],[21,0,718.75],[22,0,1437.5],[23,0,2156.25],[24,0,2875],[25,0,3593.75],[26,0,4312.5],[27,0,5031.25]]

# inputs from robot camera in degrees and mm
theta = 90
psi = 30
r = 100
marker_number = 1

calculate_distance(theta/pi*180,psi/pi*180,r)
