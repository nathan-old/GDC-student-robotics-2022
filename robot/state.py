import time
from robot_module import RobotModule
from enum import Enum
from buzzer import BuzzerManager
from config import Config
import sr.robot3 as sr
import serial.tools.list_ports

EXTRA_LONG_BEEP = 1
LONG_BEEP = 0.5
SHORT_BEEP = 0.1

STATUS_CODE_SEQUENCES = {
    "CompMode": [[sr.Note.C6, SHORT_BEEP], [sr.Note.C6, SHORT_BEEP],
                 [0, SHORT_BEEP], [sr.Note.C7, EXTRA_LONG_BEEP]],
    "DevMode": [[sr.Note.C6, SHORT_BEEP], [sr.Note.C6, SHORT_BEEP],
                [0, SHORT_BEEP], [sr.Note.C6, EXTRA_LONG_BEEP]],
    "BootDone": [[sr.Note.C6, SHORT_BEEP], [sr.Note.C6, SHORT_BEEP],
                 [sr.Note.C6, SHORT_BEEP]],
    "BootError": [[sr.Note.E6, LONG_BEEP], [0, LONG_BEEP],
                  [sr.Note.E6, LONG_BEEP], [sr.Note.E6, LONG_BEEP]],
    "InitError": [[sr.Note.G6, SHORT_BEEP], [sr.Note.E6, SHORT_BEEP],
                  [sr.Note.G6, SHORT_BEEP]],
    "BeginDiagnostics": [[sr.Note.G6, SHORT_BEEP], [sr.Note.G6, SHORT_BEEP]],
    "DiagnosticsFail": [[sr.Note.A6, SHORT_BEEP], [sr.Note.G6, LONG_BEEP]],
    "DiagnosticsPass": [[sr.Note.G6, LONG_BEEP]],
    "MainLoopError": [[sr.Note.C6, SHORT_BEEP], [sr.Note.C6, SHORT_BEEP],
                      [sr.Note.B6, SHORT_BEEP], [sr.Note.B6, SHORT_BEEP],
                      [sr.Note.G6, LONG_BEEP], [sr.Note.G6, LONG_BEEP]]
}

VERBOSE = True
VERBOSE_AUDIO = True
USE_REMOTE = True # ONLY FOR DEBUGGING/TESTING PURPOSES WILL BE REMOVED FROM CODE LONG BEFORE COMPETITION


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
        self.remoteController = None
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

        if self.LEDs is not None:
            for LED in self.LEDs:
                LED.is_enabled = False
            if code == 0:
                self.LEDs[0].is_enabled = True
            elif code == 1:
                self.LEDs[1].is_enabled = True
            elif code == -1:
                self.LEDs[2].is_enabled = True

        time.sleep(1)
        if self.buzzerManager is not None:
            self.buzzerManager.play_sequence([
                [sr.Note.C8, SHORT_BEEP], [sr.Note.C8, SHORT_BEEP],
            ])

        if self.movementManager is not None:
            self.movementManager.shutdown()
        return code

    def main(self, autoboot=False):
        self.running = True
        while self.running:
            if (self.state == States.PREBOOT):
                # Begin the boot sequence
                self.set_state(States.BOOTING)
                try:
                    self.robot = sr.Robot(auto_start=autoboot, verbose=VERBOSE)

                    self.config = Config()
                    self.config.load_disk(
                        str(self.robot.usbkey) + "/config.json")
                    self.config.save_disk(
                        str(self.robot.usbkey) + "/config.json")

                    self.LEDs = [
                        self.robot.power_board.outputs[sr.OUT_L0],
                        self.robot.power_board.outputs[sr.OUT_L1],
                        self.robot.power_board.outputs[sr.OUT_L2],
                        self.robot.power_board.outputs[sr.OUT_L3]
                    ]
                    print("Confirmed start")

                    self.buzzerManager = BuzzerManager(
                        self.robot.power_board.piezo,
                        self.config.min_time_between_buzzes)
                    if self.config.audio_debug_level > 0:
                        if (self.robot.mode == sr.DEV):
                            self.buzzerManager.play_sequence(
                                STATUS_CODE_SEQUENCES["DevMode"])
                        else:
                            self.buzzerManager.play_sequence(
                                STATUS_CODE_SEQUENCES["CompMode"])

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
                    from remote import RemoteController
                    #from pathfinder import PathfinderManager
                    #from camera import CameraManager

                    #self.mapperManager = MapperManager(self.robot, 10, 10)
                    #self.pathfinderManager = PathfinderManager(self.robot, self.mapperManager)
                    self.movementManager = MovementController(self.robot)
                    #self.cameraManager = CameraManager(self.robot)
                    self.movementManager.start_loop()
                    #self.cameraManager.start_loop();

                    ports = serial.tools.list_ports.comports()
                    if self.robot.mode == sr.DEV and USE_REMOTE:
                        print("Trying to find RemoteController")
                        for port, desc, hwid in sorted(ports):
                            target_serial_id = "7523031383335161C151"
                            serial_id = hwid.split(' ')[2][4:]
                            if VERBOSE:
                                print("Device: Port={}, Description={}, HardwareID={}".format(port, desc, hwid))
                            if serial_id == target_serial_id:
                                self.remoteController = RemoteController(self.robot, self.movementManager, serial_id, port, 9600, self.shutdown)
                                break     

                    if self.remoteController is None:
                        print("No remote controller found")                   
                except Exception as e:
                    self.errors.append(e)
                    self.failure_state = States.INITIALIZING
                    self.set_state(States.ERROR)
                    continue
                # we have initialized our modules, now we can start the diagnostics
                self.set_state(States.RUNNING_DIAGNOTIC)
            if (self.state == States.RUNNING_DIAGNOTIC):
                if self.config.audio_debug_level > 0:
                    self.buzzerManager.play_sequence(
                        STATUS_CODE_SEQUENCES["BeginDiagnostics"])
                try:
                    # TODO: this is where we would start the diagnostics, for now we just do a dance
                    self.movementManager.turn_angle(90)
                    self.movementManager.turn_angle(-90)
                    self.movementManager.wait_for_queue_clearance()
                except Exception as e:
                    self.errors.append(e)
                    self.failure_state = States.RUNNING_DIAGNOTIC
                    self.set_state(States.ERROR)
                    continue
                # we have finished the diagnostics, now we can go back to idle
                self.buzzerManager.play_sequence(
                    STATUS_CODE_SEQUENCES["DiagnosticsPass"])
                self.set_state(States.IDLE)
            if (self.state == States.IDLE):
                try:
                    print("ALL DONE, REACHED IDLE STATE")

                    self.buzzerManager.play_sequence(
                        [i, 0.05] for i in range(1000, 5000, 150))
                    if USE_REMOTE:
                        self.remoteController.start_listening()
                    self.movementManager.wait_for_queue_clearance()
                    # Rc controller will change the state to Finished and will shutdown when button pressed 
                except Exception as e:
                    self.errors.append(e)
                    self.failure_state = States.IDLE
                    self.set_state(States.ERROR)
                    continue

            if (self.state == States.ERROR):
                print("entered error state during state: " +
                      str(self.failure_state))
                if VERBOSE:
                    print("errors:")
                    for i in range(len(self.errors)):
                        print("({}) {}".format(i, self.errors[i]))
                if (self.failure_state == States.BOOTING):
                    for i in range(5):
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["BootError"])
                        time.sleep(1.5)
                    return self.shutdown(1)
                if (self.failure_state == States.INITIALIZING):
                    for i in range(5):
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["InitError"])
                        time.sleep(1.5)
                    return self.shutdown(1)
                elif (self.failure_state == States.RUNNING_DIAGNOTIC):
                    for i in range(5):
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["DiagnosticsFail"])
                        time.sleep(1.5)
                    return self.shutdown(1)
                elif (self.failure_state == States.IDLE):
                    for _ in range(5):
                        self.buzzerManager.play_sequence(
                            STATUS_CODE_SEQUENCES["MainLoopError"])
                        time.sleep(1.5)
                    return self.shutdown(1)
                elif (self.failure_state == States.ERROR):
                    return self.shutdown(-1)
                elif (self.failure_state == States.FINISHED):
                    return self.shutdown(0)
                else:
                    print("Unknown failure state: " + str(self.failure_state))
                    return self.shutdown(-1)
