from threading import Thread
from sr.robot3 import *
import time
import pathfinder
from movement import MovementMaster, RouteCommands
from grabber import Arduino, Communicate
from position import Position
from constants import can_locations, obstacles

# Config varibles
RobotInfo_Enable = True
Instructions_Enable = True
PathFinder_Enable = False
Grabber_Enable = True
Set_Bearing_Enable = False
Goto_Set_Position = False
Set_Position = (50, 50)  # X, Y (In mm)
route_paths = ["route.txt"]

# setup module holder global varibles
arduino = None
RUGGEDUINO_ID = None
R = None
com = None
position_finder = None
movement = None
routecommands = None
class RobotController():
    def __init__(self, robot, arduino, arduino_communication, position_finder, movement_controller, route_commands, set_bearing, use_instructions, use_pathfinder, route_paths):
        self.robot = robot
        self.arduino = arduino
        self.arduino_communication = arduino_communication
        self.position_finder = position_finder
        self.movement_controller = movement_controller
        self.route_commands = route_commands
        self.set_bearing = set_bearing
        self.use_instructions = use_instructions
        self.use_pathfinder = use_pathfinder
        self.route_paths = route_paths

        self.position = None
        self.updater_thread = Thread(target=self.updater_thread_logic)
        self.instructions = []

    def go(self):
        self.prebutton_setup()
        print("[INFO] Prebutton press setup complete, waiting for button press")
        R.wait_start()
        print("[INFO] Button pressed, running main code")
        self.after_button()

    def updater_thread_logic(self):
        print("[INFO] Started updater thread")
        image_index = 0
        while True:
            # TODO: Try and find our location, if we can find it save to the json file. Also save other data (like current action being performed, are we moving etc)
            R.camera.save(str(R.usbkey/str(image_index)) + '.png')
            image_index += 1

    def prebutton_setup(self):
        R.servo_board.servos[0].position = 0  # To give power output
        R.camera.see()  # start camera stream now (so its quicker to access mid-game)
        print("[INFO] Starting Zone: {}".format(self.robot.zone))
        print("[INFO] Arena: {}".format(self.robot.arena))
        print("[INFO] Mode: {}".format(self.r.mode))
        print("[INFO] Zone: {}".format(self.robot.zone))
        print("[INFO] Robot arm radius: {}".format(
            self.movement_controller.arm_radius))
        print("[INFO] Wheel circumference: {}".format(
            self.movement_controller.wheel_circumference))
        if self.use_instructions:
            print("Instructions enabled, loading {} routes".format(
                len(self.route_paths)))
            global_route = []
            loaded_routes = 0
            for route_file_path in self.route_paths:
                try:
                    with open(route_file_path, 'r') as route_file:
                        route_unsplit = route_file.readlines()
                        route_split = []
                        for i in range(len(route_unsplit)):
                            route_split.append(route_unsplit[i].split(', '))
                        global_route.extend(global_route)
                        loaded_routes += 1
                except Exception as error:
                    print(
                        "[ERROR] Failed to load route {}, got exception {}. THIS MAY/WILL CAUSE UNEXPECTED BEHAVIOR".format(route_file_path, error))

            print("[INFO] Loaded all {} of {} routes, totaling {} instructions".format(
                loaded_routes, len(self.route_paths), len(global_route)))
            self.instructions = global_route
        print("[INFO] Starting updater thread")
        self.updater_thread.start()
        print("[INFO] Prebutton press startup logic done")

    def start_bearing(self):
        if int(R.zone) == 0:
            return 105
        elif int(R.zone) == 1:
            return 195
        elif int(R.zone) == 2:
            return 285
        elif int(R.zone) == 3:
            return 15

    def after_button(self):
        if self.set_bearing:
            self.set_bearing(self.start_beraing())

        if self.use_pathfinder:
            for i in range(3):
                self.position = self.get_position()
                if self.position != None:
                    bearing = self.position[1]
                    tolerance = 5
                    if 90 - tolerance > bearing or bearing > 90 + tolerance:
                        movement.rotate(float(bearing-90), 0.3)
                else:
                    print('[WARN] Cannot determine pathfinding location, cannot pathfind ')

            if self.position != None:
                place = self.position[0]
                route_found = pathfinder.PathFind(
                    place, can_locations, obstacles)

                route_found.append(['grab', '0'])
                self.route_commands.follow(route_found)
                print('[INFO] found a route')
            else:
                print('[WARN] cannot get pathfinding to work - insufficient position data')
        elif self.use_instructions:
            print("[INFO] Use instructions enabled, following {} loaded route commands".format(
                len(self.instructions)))
            self.route_commands.follow(self.instructions)

    def get_position(self):
        position = position_finder.try_untill_find()
        if position != None:
            print('[INFO] Position: {}, Angle: {}'.format(
                round(position[0][0], 2), round(position[0][1], 2)))
            return position
        else:
            print('[WARN] Cannot find a position')
            return None

    def set_bearing(self, bearing):
        position = self.get_position()
        turn_angle = position[1] - bearing
        movement.rotate(turn_angle, 0.3)



def run():
    global R, arduino, com, position_finder, movement, routecommands, RUGGEDUINO_ID
    arduino = Arduino()
    RUGGEDUINO_ID = arduino.Setup()
    R = Robot(auto_start=True, verbose=True,
              ignored_ruggeduinos=[RUGGEDUINO_ID])
    com = Communicate(arduino.Start(R, RUGGEDUINO_ID))
    position_finder = Position(R)
    movement = MovementMaster(R)
    routecommands = RouteCommands(
        R, movement, Grabber_Enable, com, position_finder)
    # Create the robot controller class
    robot_controller = RobotController(R, arduino, com, position_finder, movement, routecommands,
                                       Set_Bearing_Enable, Instructions_Enable, PathFinder_Enable, route_paths)
    robot_controller.go()

run()
