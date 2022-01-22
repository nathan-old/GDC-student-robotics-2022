import json


class Config():

    def __init__(self):
        self.version = "0.0.1"

        self.audio_debug_level = 2  # 0: no debug, 1: debug error, info, 2: debug error, info, trace
        self.motor_boards = ['SR0PJ1W',
                             'SR0VG1M']  # the id of the motor boards
        self.wheel_count = 3  # number of wheels on the robot
        self.turn_wheel_index = 0  # the index of the wheel that is used for turning

        self.wheel_circumference = 0.15  # the circumference of the wheels in meters
        self.arm_radius = 0.18  # the radius of the arm in meters
        self.wrap_angles = False  # if true, the angle will be wrapped to be between -180 and 180
        self.min_time_between_buzzes = 0.1  # the minimum time between buzzes in seconds

    def from_json(self, json):
        # decode the json
        decoded_json = json.loads(json)
        # set the values
        try:
            self.version = decoded_json['version']
            self.audio_debug_level = decoded_json['audio_debug_level']
            self.motor_boards = decoded_json['motor_boards']
            self.wheel_count = decoded_json['wheel_count']
            self.turn_wheel_index = decoded_json['turn_wheel_index']
            self.wheel_circumference = decoded_json['wheel_circumference']
            self.arm_radius = decoded_json['arm_radius']
            self.wrap_angles = decoded_json['wrap_angles']
            self.min_time_between_buzzes = decoded_json[
                'min_time_between_buzzes']
        except KeyError as e:
            raise Exception("Invalid config file: " + str(e))
        except:
            print("Error: Could not decode the config file")

    def to_json(self):
        # create a dictionary
        config_dict = {
            'version': self.version,
            'audio_debug_level': self.audio_debug_level,
            'motor_boards': self.motor_boards,
            'wheel_count': self.wheel_count,
            'turn_wheel_index': self.turn_wheel_index,
            'wheel_circumference': self.wheel_circumference,
            'arm_radius': self.arm_radius,
            'wrap_angles': self.wrap_angles,
            'min_time_between_buzzes': self.min_time_between_buzzes
        }
        # encode the dictionary
        encoded_json = json.dumps(config_dict)
        # return the encoded json
        return encoded_json

    def save_disk(self, path):
        try:
            # get the json
            json = self.to_json()
            # save the json to disk
            with open(path, 'w+') as outfile:
                outfile.write(json)
        except:
            print("Error: Could not encode the config file")

    def load_disk(self, path):
        try:
            # open the file
            with open(path, 'r') as infile:
                # read the file
                json = infile.read()
                # decode the json
                self.from_json(json)
        except FileNotFoundError:
            print("Error: Could not find the config file")
        except:
            print("Error: Could not decode the config file")
