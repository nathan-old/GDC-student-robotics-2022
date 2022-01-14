import threading
import time
from robot_module import RobotModule
from sr.robot3 import Note


class BuzzerManager(RobotModule):

    def __init__(self, buzzer, min_time_between_buzzes=0.2):
        super().__init__("buzzer", "Buzzer manager", "0.0.1", "0.0.0", "1.0.0")
        self.buzzer = buzzer
        self.min_time_between_buzzes = min_time_between_buzzes

    def _playtone(self, tone, duration=0.13):

        if (duration * 1000) > 60000:
            print("Playing tone: " + str(tone) + " for " + str(duration) +
                  " which is over 65secconds (abort)")
            return
        self.buzzer.buzz(duration, tone)

    def play_sequence(
        self, sequence
    ):  # sequence format: [ [tone, duration], [tone, duration], ... ]
        for part in sequence:
            if part[0] <= 20: # inauble so used as a "sleep"
                time.sleep(part[1])
            else:
                self._playtone(part[0], part[1])
                time.sleep(part[1])
            time.sleep(self.min_time_between_buzzes)