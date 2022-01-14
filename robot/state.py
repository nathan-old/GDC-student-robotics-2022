import threading
import time
from robot_module import RobotModule
from enum import Enum
from buzzer import BuzzerManager
import sr.robot3 as sr

STATUS_CODE_SEQUENCES = {
    "PB_C": [[sr.Note.C6, 0.25], [sr.Note.C6, 0.2], [0, 0.25],
             [sr.Note.C7, 1]],  # Preboot comp mode
    "PB_D": [[sr.Note.C6, 0.25], [sr.Note.C6, 0.2], [0, 0.25],
             [sr.Note.C6, 1]],  # Preboot dev mode
    "BC": [[sr.Note.C6, 0.5], [0, 0.25], [sr.Note.C6, 0.5], [0, 0.25],
           [sr.Note.C6, 0.5]],  # Boot complete
    "BE": [[sr.Note.E6, 0.5], [0, 0.5], [sr.Note.E6, 0.5], [0, 0.25],
           [sr.Note.E6, 0.5], [0, 1]],  # Boot error
    "IE": [[sr.Note.G6, 0.1], [sr.Note.E6, 0.1], [sr.Note.G6, 0.1],
           [sr.Note.E6, 0.1], [sr.Note.G6, 0.25], [sr.Note.G6, 0.25],
           [0, 1]],  # Initialization error
    "BD": [[sr.Note.G6, 0.1], [sr.Note.G6, 0.1]],  # Begining Diagnostics
    "DF": [[i, 0.2] for i in range(4000, 1000, 250)],  # Diagnostic failure
    "DG": [[i, 0.2] for i in range(1000, 4000, 250)],  # Diagnostic good
    "MF": [[sr.Note.C6, 0.1], [sr.Note.C6, 0.1], [sr.Note.B6, 0.1],
           [sr.Note.B6, 0.1], [sr.Note.G6, 0.25], [sr.Note.G6, 0.25],
           [0, 1]],  # main function error
}

VERBOSE = True
VERBOSE_AUDIO = True

class States(Enum):
    PREBOOT = -1,
    BOOTING = 0,
    INITIALIZING = 1,
    RUNNING_DIAGNOTIC = 2,
    IDLE = 3,
    ERROR = 4,
    FINISHED = 5,


class StateManager(RobotModule):

    def __init__(self):
        super().__init__("state_manager",
                         "State manager, the main loop and logic master",
                         "0.0.1", "0.0.0", "1.0.0")
        self.running = False
        self.state = States.PREBOOT
        self.robot = None
        self.pathfinderManager = None
        self.mapperManager = None
        self.movementManager = None
        self.cameraManager = None
        self.buzzerManager = None
        self.failure_state = None
        self.LEDs = []
        self.errors = []

    def set_state(self, state):
        self.state = state
        if VERBOSE:
            print("State changed to: " + str(state))

    def shutdown(
        self,
        code=0
    ):  # -1 restart, 0 normal shutdown, 1 error shutdown, 2 something that should never happen (logic error)
        if not self.running:
            return
        self.running = False

        # TODO: shutdown all modules
        return code

    def led_sweep(self, count=5):
        for _ in range(count):
            for LED in self.LEDs:
                LED.is_enabled = not LED.is_enabled
                time.sleep(0.05)
            self.LEDs.reverse()
            time.sleep(0.25)

    def main(self, autoboot=False):
        self.running = True
        while self.running:
            if (self.state == States.PREBOOT):
                # Begin the boot sequence
                self.set_state(States.BOOTING)
                try:
                    self.robot = sr.Robot(auto_start=autoboot, verbose=VERBOSE)
                    self.robot.wait_start()  # wait for the robot to start
                    self.robot._log_discovered_boards
                    self.LEDs = [self.robot.power_board.outputs[sr.OUT_L0], self.robot.power_board.outputs[sr.OUT_L1], self.robot.power_board.outputs[sr.OUT_L2], self.robot.power_board.outputs[sr.OUT_L3]]
                    print("Confirmed start")

                    self.buzzerManager = BuzzerManager(
                        self.robot.power_board.piezo)
                    self.buzzerManager.play_sequence([[500, 0.5]])
                    self.led_sweep(5)

                    if (self.robot.mode == sr.DEV):
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["PB_D"])
                    else:
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["PB_C"])

                    self.buzzerManager.play_sequence(
                        STATUS_CODE_SEQUENCES["BC"])
                except Exception as e:
                    self.errors.append(e)
                    self.failure_state = States.BOOTING
                    self.set_state(States.ERROR)
                    continue
                # the robot has now started and we can initialize our wrapper modules/classes
                self.set_state(States.INITIALIZING)
            if (self.state == States.INITIALIZING):
                try:
                    #from mapper import MapperManager
                    from movement import MovementController
                    #from pathfinder import PathfinderManager
                    #from camera import CameraManager

                    #self.mapperManager = MapperManager(self.robot, 10, 10)
                    #self.pathfinderManager = PathfinderManager(self.robot, self.mapperManager)
                    self.movementManager = MovementController(self.robot)
                    #self.cameraManager = CameraManager(self.robot)

                    self.movementManager.start_loop();
                    #self.cameraManager.start_loop();
                except Exception as e:
                    self.errors.append(e)
                    self.failure_state = States.INITIALIZING
                    self.set_state(States.ERROR)
                    continue;
                # we have initialized our modules, now we can start the diagnostics
                self.buzzerManager.play_sequence([[sr.Note.C6, 0.5]])
                self.set_state(States.RUNNING_DIAGNOTIC)
            if (self.state == States.RUNNING_DIAGNOTIC):
                if VERBOSE_AUDIO:
                    self.buzzerManager.play_sequence(
                        STATUS_CODE_SEQUENCES["BD"])
                try:
                    # TODO: this is where we would start the diagnostics, for now we just do a dance
                   # self.movementManager.turn_angle(90)
                    #self.movementManager.turn_angle(-90)
                    #self.movementManager.wait_for_queue_clearance()
                    None;
                except Exception as e:
                    self.errors.append(e)
                    self.failure_state = States.RUNNING_DIAGNOTIC
                    self.set_state(States.ERROR)
                    continue
                # we have finished the diagnostics, now we can go back to idle
                self.buzzerManager.play_sequence(STATUS_CODE_SEQUENCES["DG"])
                self.set_state(States.IDLE)
            if (self.state == States.IDLE):
                try:
                    self.buzzerManager.play_sequence([
                        [3000, 0.1, 4000, 0.4, 6000, 1]
                    ])
                    # we have finished the setup, start the main loop
                    print("ALL DONE, REACHED IDLE STATE")
                    with open('instructions.txt', 'r') as file_:
                        instructions = file_.readlines()

                    for i in range(len(instructions)):
                        instructions[i] = instructions[i].strip().split(',')
                    for i in range(len(instructions)):
                        if instructions[i][0] == 'forward':
                            self.movementManager.forward(
                                int(instructions[i][1]))
                        if instructions[i][0] == 'turn':
                            self.movementManager.turn_angle(
                                int(instructions[i][1]))
                        if instructions[i][0] == 'wait':
                            self.movementManager.wait_for_queue_clearance()
                        if instructions[i][0] == 'take_picture':
                            self.robot.camera.save(
                                instructions[i][1] +
                                ".jpg")  # TODO: use the camera manager
                        if instructions[i][0] == 'move':
                            self.movementManager.move(int(instructions[i][1]),
                                                      int(instructions[i][2]))
                        if instructions[i][0] == 'exit':
                            self.shutdown(int(instructions[i][1]))

                    self.movementManager.wait_for_queue_clearance()
                    return self.shutdown(0)  # all went well!
                except Exception as e:
                    self.errors.append(e)
                    self.failure_state = States.IDLE
                    self.set_state(States.ERROR)
                    continue

            if (self.state == States.ERROR):
                print("entered error state during state: " + str(self.state))
                if VERBOSE:
                    print("errors:")
                    for i in range(len(self.errors)):
                        print("({}) {}".format(i, self.errors[i]))
                if (self.failure_state == States.BOOTING):
                    for i in range(5):
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["BE"])
                        time.sleep(1.5)
                if (self.failure_state == States.INITIALIZING):
                    for i in range(5):
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["IE"])
                        time.sleep(1.5)
                elif (self.failure_state == States.RUNNING_DIAGNOTIC):
                    self.buzzerManager.play_sequence(
                        STATUS_CODE_SEQUENCES["DF"])
                elif (self.failure_state == States.IDLE):
                    sweeper_thread = threading.Thread(self.led_sweep(350)).start()
                    for _ in range(25):
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["MF"])
                        time.sleep(1.5)
                    return 1
                elif (self.failure_state == States.ERROR):
                    return -1
                    # Something is very wrong
                elif (self.failure_state == States.FINISHED):
                    return 0
                    # Everything is fine, we are done
                else:
                    print("Unknown failure state: " + str(self.failure_state))

                time.sleep(2)
                self.set_state(States.FINISHED)

                return 0
