import time
from robot_module import RobotModule
from sr.robot3 import Note

class BuzzerManager(RobotModule):
    def __init__(self, buzzer, min_time_between_buzzes=0.1):
        self.buzzer = buzzer
        self.min_time_between_buzzes = min_time_between_buzzes
    
    def playtone(self, tone, duration=0.13):
        self.buzzer.buzz(duration, tone)
    
    def play_sequence(self, sequence): # sequence format: [ [tone, duration], [tone, duration], ... ]
        for i in range(len(sequence)):
            if (sequence[i][0] == 'pause'):
                time.sleep(sequence[i][1])
            else:
                self.playtone(sequence[i][0], sequence[i][1])
            time.sleep(self.min_time_between_buzzes)
    
    def time_to_play(self, sequence):
        time_to_play = 0
        for i in range(len(sequence)):
            time_to_play += sequence[i][1] + self.min_time_between_buzzes
        # better to wait a bit longer than necessary
        time_to_play += self.min_time_between_buzzes
        return time_to_play