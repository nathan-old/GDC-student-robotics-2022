import json
import os
from sqlite3 import Timestamp
from threading import Thread
from sr.robot3 import *
import time
import pathfinder
from movement import MovementMaster, RouteCommands
from grabber import Arduino, Communicate
from position import Position
from constants import can_locations, obstacles
import utils

# Config varibles
RobotInfo_Enable = True
Instructions_Enable = True
PathFinder_Enable = False
Grabber_Enable = True
Set_Bearing_Enable = False
route_paths = ["routes/can_one_pick_flip.route",
               "routes/left_sweep.route", "routes/right_sweep.route"]
# Should we create a debug log even when the calculated position is None?
debug_log_even_with_no_pos = True

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
        self.robot.wait_start()
        print("[INFO] Button pressed, running main code")
        self.after_button()

    def updater_thread_logic(self):
        print("[INFO] Started updater thread")
        debug_folder_path = str(self.robot.usbkey) + "/debug"
        print("[INFO][UPDATER] Checking if debug folder exists at path: {}".format(
            debug_folder_path))
        if not os.path.exists(debug_folder_path):
            os.makedirs(debug_folder_path)
            print("[INFO][UPDATER] Debug folder did not exist; Created")
        else:
            print("[INFO][UPDATER] Debug folder already exists")
        image_folder_path = str(self.robot.usbkey) + "/images"
        print("[INFO][UPDATER] Checking if images folder exists at path: {}".format(
            image_folder_path))
        if not os.path.exists(image_folder_path):
            os.makedirs(image_folder_path)
            print("[INFO][UPDATER] Photos folder did not exist; Created")
        else:
            print("[INFO][UPDATER] Photos folder already exists")
        image_index = 0
        print("[INFO][UPDATER] Saving aggregated route to {}/total_route.route".format(debug_folder_path))
        with open("{}/total_route.route".format(debug_folder_path), "w") as total_route_file:
            for instruction in self.route_commands.route:
                total_route_file.write(','.join(instruction) + "\n")
        while True:
            self.robot.camera.save(image_folder_path + "/" +
                                   str(image_index) + '.png')
            image_index += 1
            try_calc_pos = None
            if not self.position_finder.block:
                try_calc_pos = self.position_finder.get_pos()
            if try_calc_pos or debug_log_even_with_no_pos:
                is_moving = self.movement_controller.moving
                with open(debug_folder_path + "/" + str(time.time()) + '-debug.json', "w") as debug_file:
                    content_dict = {
                        "calculated_position": try_calc_pos,
                        "is_moving": is_moving,
                        "timestamp": time.time(),
                        "current_route_step_command": self.route_commands.current_command(),
                        "current_route_step_index": self.route_commands.current_command_index
                    }
                    content = json.dumps(content_dict)
                    debug_file.write(content)

    def prebutton_setup(self):
        self.robot.servo_board.servos[0].position = 0  # To give power output
        self.robot.camera.see()  # start camera stream now (so its quicker to access mid-game)
        print("[INFO] Starting Zone: {}".format(self.robot.zone))
        print("[INFO] Arena: {}".format(self.robot.arena))
        print("[INFO] Mode: {}".format(self.robot.mode))
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
                        print("[DEBUG] route readlines len: {}".format(
                            len(route_unsplit)))
                        route_split = []
                        for i in range(len(route_unsplit)):
                            route_split.append(route_unsplit[i].split(', '))
                        global_route.extend(route_split)
                        loaded_routes += 1
                except Exception as error:
                    print(
                        "[ERROR] Failed to load route {}, got exception {}. THIS MAY/WILL CAUSE UNEXPECTED BEHAVIOR".format(route_file_path, error))

            print("[INFO] Loaded all {} of {} routes, totaling {} instructions".format(
                loaded_routes, len(self.route_paths), len(global_route)))
            self.instructions = global_route
            # Purley for getting the current route number and not crashing
            self.route_commands.set_route(self.instructions)
        print("[INFO] Starting updater thread")
        self.updater_thread.start()
        print("[INFO] Prebutton press startup logic done")

    def start_bearing(self):
        return utils.calc_target_bearing(self.robot)

    def after_button(self):
        if self.set_bearing:
            self.call_set_bearing(self.start_bearing(), tries=1)

        if self.use_pathfinder:
            for i in range(3):
                self.position = self.get_position()
                if self.position != None:
                    bearing = self.position[1]
                    tolerance = 5
                    if 90 - tolerance > bearing or bearing > 90 + tolerance:
                        movement.rotate(float(bearing-90), 0.3)
                else:
                    print(
                        '[WARN] Cannot determine pathfinding location, cannot pathfind ')

            if self.position != None:
                place = self.position[0]
                route_found = pathfinder.PathFind(
                    place, can_locations, obstacles)

                route_found.append(['grab', '0'])
                self.route_commands.follow(route_found)
                print('[INFO] found a route')
            else:
                print(
                    '[WARN] cannot get pathfinding to work - insufficient position data')
        elif self.use_instructions:
            print("[INFO] Use instructions enabled, following {} loaded route commands".format(
                len(self.instructions)))
            self.route_commands.follow(self.instructions)

    def get_position(self):
        return self.movement_controller.pos_get(self.position_finder)

    def call_set_bearing(self, bearing, tries=3):
        self.movement_controller.set_bearing(
            self.position_finder, target=bearing, tries=tries)


def run():
    global R, arduino, com, position_finder, movement, routecommands, RUGGEDUINO_ID
    arduino = Arduino()
    RUGGEDUINO_ID = arduino.Setup()
    R = Robot(auto_start=True, verbose=True,
              ignored_ruggeduinos=[RUGGEDUINO_ID])
    com = Communicate(arduino.Start(R, RUGGEDUINO_ID))
    movement = MovementMaster(R)
    position_finder = Position(R, movement)
    routecommands = RouteCommands(
        R, movement, Grabber_Enable, com, position_finder)
    # Create the robot controller class
    robot_controller = RobotController(R, arduino, com, position_finder, movement, routecommands,
                                       Set_Bearing_Enable, Instructions_Enable, PathFinder_Enable, route_paths)
    robot_controller.go()


run()
