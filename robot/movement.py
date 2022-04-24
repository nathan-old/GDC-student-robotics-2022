import time
import math
from sr.robot3 import *
import utils


class MovementMaster():
    def __init__(self, robot):
        ''' Robot dimension constants'''
        print('[INIT] Movement Master initialised')
        R = robot
        self.R = robot
        self.arm_radius = 0.25
        self.wheel_circumference = 0.3299
        self.motors = [R.motor_boards['SR0PJ1W'].motors[0], R.motor_boards['SR0VG1M'].motors[0],
                       R.motor_boards['SR0VG1M'].motors[1]]  # m0 and m1 are front motors
        self.intergrated_distance_acceleration = 0.199569041087827
        self.rps_constant = 1.54090799872
        self.speed = self.wheel_circumference * self.rps_constant
        self.moving = False

    def rotate(self, angle, power):
        ''' function to turn the robot'''
        print('Turning - ' + str(angle) + ' at power - ' + str(power))
        angle /= 1.13
        negative = 1
        if angle < 0:
            negative = -1
            angle *= -1
        speed = self.wheel_circumference * ((4.6778*(power**6) - 88.46*(power**5) - 9.1788*(
            power**4) + 82.827*(power**3) + 5.6922*(power**2) + 97.405*(power))/60)
        time_to_move = (angle * ((math.pi * 2 * self.arm_radius) / 360))/speed
        self.moving = True
        for wheel in self.motors:
            wheel.power = power*negative
        time.sleep(time_to_move)
        self.moving = False
        for wheel in self.motors:
            wheel.power = BRAKE

    def sideways(self, distance, front_wheels=[2, 1, 0]):
        print('sideways - ' + str(distance))
        distance /= 0.966982143
        negative = 1
        if distance < 0:
            negative = -1
            distance *= -1
        power = 0.63
        speed = self.wheel_circumference * ((4.6778*(power**6) - 88.46*(power**5) - 9.1788*(
            power**4) + 82.827*(power**3) + 5.6922*(power**2) + 97.405*(power))/60)
        time_to_move = distance/speed
        self.moving = True
        self.motors[front_wheels[0]].power = power * negative
        self.motors[front_wheels[1]].power = -0.35 * negative
        self.motors[front_wheels[2]].power = -0.35 * negative
        time.sleep(time_to_move)
        self.moving = False
        for wheel in self.motors:
            wheel.power = BRAKE

    def forwards(self, distance, front_wheels=[0, 1]):
        '''function to move the robot forwards and backwards
        Distance: the distance you wish to move the robot
        front_wheels: the index of the wheels you wish to use (normally 0 and 1)'''
        print('forwards - ' + str(distance))
        distance *= 0.9293
        distance /= 1.11
        self.negative = 1
        if distance < 0:
            self.negative = -1
            distance *= -1
        if distance > (self.intergrated_distance_acceleration):
            distance = self.accelerate_forwards(
                distance, self.negative, front_wheels)
            time_to_move = distance/self.speed
            self.moving = True
            self.motors[front_wheels[0]].power = 0.8 * self.negative
            self.motors[front_wheels[1]].power = 0.8 * -1 * self.negative
            time.sleep(time_to_move)
            self.moving = False
            #self.decelerate_forwards(self.negative, front_wheels)
            self.motors[front_wheels[0]].power = BRAKE
            self.motors[front_wheels[1]].power = BRAKE
        else:
            time_to_move = distance/(self.wheel_circumference * 0.528070517)
            self.moving = True
            self.motors[front_wheels[0]].power = 0.3 * self.negative
            self.motors[front_wheels[1]].power = 0.3 * -1 * self.negative
            time.sleep(time_to_move)
            self.moving = False
            self.motors[front_wheels[0]].power = BRAKE
            self.motors[front_wheels[1]].power = BRAKE

    def accelerate_forwards(self, distance, negative, front_wheels):
        ''' The function to make the motors accelerate the robot
        Distance: the distance for the wheels to go
        Negative: to tell us to reverse motors
        front_wheels: the index of the wheels you wish to use (normally 0 and 1)'''
        num_of_changes = 8
        time_to_accelerate = 0.75
        time_per_step = time_to_accelerate/num_of_changes

        for i in range(1, num_of_changes+1, 2):
            self.motors[front_wheels[0]].power = (
                0.8/num_of_changes)*i * negative
            self.motors[front_wheels[1]].power = (
                0.8/num_of_changes)*i * -1 * negative
            time.sleep(time_per_step)
            self.motors[front_wheels[1]].power = (
                (0.8/num_of_changes)*(i+1) * -1 * negative)
            self.motors[front_wheels[0]].power = (
                (0.8/num_of_changes)*(i+1) * negative)
            time.sleep(time_per_step)

        return distance - self.intergrated_distance_acceleration

    def curve(self, radius, angle, power=0.3):
        if angle < 0:
            angle *= -1
            negative = -1
        else:
            negative = 1
        speed = self.wheel_circumference * ((4.6778*(power**6) - 88.46*(power**5) - 9.1788*(
            power**4) + 82.827*(power**3) + 5.6922*(power**2) + 97.405*(power))/60)
        distance = math.pi * (2 * radius) * (float(angle)/360.0)
        print(distance, speed, angle)
        time_to_drive = distance / speed
        rotatanal_speed = (
            angle * ((math.pi * 2 * self.arm_radius) / 360))/time_to_drive
        rotatanal_speed *= negative
        print(speed + rotatanal_speed, (speed * -1) +
              rotatanal_speed, rotatanal_speed)
        self.moving = True
        self.motors[0].power = speed + rotatanal_speed
        self.motors[1].power = (speed * -1) + rotatanal_speed
        self.motors[2].power = rotatanal_speed
        time.sleep(time_to_drive)
        self.moving = False
        self.motors[0].power = BRAKE
        self.motors[1].power = BRAKE
        self.motors[2].power = BRAKE

    def pos_get(self, position_finder):
        position = position_finder.try_untill_find()
        if position != None:
            print('[INFO] Position: {}, Angle: {}'.format(
                round(position[0][0], 2), round(position[0][1], 2)))
            return position
        else:
            print('[WARN] Cannot find a position')
            return None

    def set_bearing(self, position_finder, tries=3, target=None):
        target_bearing = utils.calc_target_bearing(
            self.R) if not target else utils.bearing_of_zero_for_zones(self.R.zone) + target
        if target_bearing > 360:
            target_bearing -= 360
        if target_bearing < 0:
            target_bearing += 360
        print("[INFO] (movement controller) set bearing, target bearing: {}".format(
            target_bearing))
        for i in range(tries):
            position = self.pos_get(position_finder)
            if position is None:
                print(
                    "Did not find position on set bearing iteration {}/{}, retrying".format(i + 1, tries))
                continue

            turn_angle = target_bearing - position[1]
            self.rotate(turn_angle, 0.3)
            break
    
    def set_position_center(self, position_finder):
        print("[INFO] Starting set position center")
        for i in range(3):
            start_pos = self.pos_get(position_finder)
            if start_pos is None:
                continue
            print("[INFO] Set position center found pos: {}".format(start_pos))
            end_pos = []
            start_zone = 2 #self.R.zone
            if start_zone == 0:
                end_pos = [0.5,5.25]
            elif start_zone == 1:
                end_pos = [5.25,5.25]
            elif start_zone == 2:
                end_pos = [5.25,0.5]
            else:
                end_pos = [0.5,0.5]
            
            print("Selected start position of {} (corner center) based on start zone of: {}".format(start_pos, self.R.zone))
            
            x_change = end_pos[0] - start_pos[0][0]
            y_change = end_pos[1] - start_pos[0][1]
            print("[INFO] X change: {}, Y change: {}".format(x_change, y_change))
            distance = math.sqrt((x_change**2)+(y_change**2))
            target_bearing = math.atan2(x_change,y_change)
            print("[INFO] Calculated distance of {} with target bearing of {} from x,y delta".foramt(distance, target_bearing))
            turn_angle = target_bearing - start_pos[1]
            print("[INFO] turn angle: {}".format(turn_angle))
            self.rotate(turn_angle, 0.3)
            self.forwards(distance)
            break
        print("[INFO] Exiting from set position center")


class RouteCommands():
    def __init__(self, R, movement, Grabber_Enable, com, position_finder):
        self.movement = movement
        self.Grabber_Enable = Grabber_Enable
        self.com = com
        self.position_finder = position_finder
        self.R = R
        self.current_command_index = -1
        self.route = []
        print('[INIT] Route follower initialised')

    def set_route(self, route):
        self.route = route

    def current_command(self):
        if self.current_command_index == -1:
            return "not started route yet"
        if self.current_command_index > len(self.route) - 1:
            return "finished route"
        return self.route[self.current_command_index]

    def follow(self, route):
        self.route = route
        self.current_command_index = 0
        for i in route:
            try:
                # if len(i) < 2:
                #    print('[WARN] Invalid instruction {} on line '.format(i) +
                #       str(route.index(i)+1) + ' -- skipping')
                #
                #    continue

                # Remove newline char
                for i_sub_i in range(len(i)):
                    i[i_sub_i] = i[i_sub_i].replace("\n", "")

                if i[0] == 'forwards':
                    if len(i) == 3:
                        if i[2] == 'Plough\n' or i[2] == 'Plough':
                            front = [2, 0]
                        elif i[2] == 'Empty\n' or i[2] == 'Empty':
                            front = [1, 2]
                        else:
                            front = [0, 1]
                    else:
                        front = [0, 1]
                    self.movement.forwards(float(i[1]), front)  # [2,0]
                elif i[0] == 'bearing':
                    print("Running set bearing")
                    if len(i) == 2:
                        self.movement.set_bearing(self.position_finder, target=float(i[1]))
                    else:
                        self.movement.set_bearing(self.position_finder)
                elif i[0] == "set_position_center":
                    print("Running set position center")
                    self.movement.set_position_center(self.position_finder)
                elif i[0] == 'beep':
                    print("Beeping (D6, {} seccond(s))".format(i[1]))
                    self.R.power_board.piezo.buzz(float(i[1]), Note.D6)
                    if len(i) == 3:
                        time.sleep(float(i[1]))
                elif i[0] == 'turn':
                    self.movement.rotate(float(i[1]), 0.3)
                elif i[0] == 'sleep':
                    print("Sleeping for {} secconds".format(i[1]))
                    time.sleep(float(i[1]))
                elif i[0] == 'grab' and self.Grabber_Enable:
                    print("Grabbing")
                    self.com.Grab()
                elif i[0] == 'curve':
                    self.movement.curve(float(i[1]), float(i[2]), 0.3)
                elif i[0] == 'wait':
                    self.R.wait_start()
                elif i[0] == 'sideways':
                    if len(i) == 3:
                        if i[2] == 'Plough\n' or i[2] == 'Plough':
                            front = [1, 0, 2]
                        elif i[2] == 'Empty\n' or i[2] == 'Empty':
                            front = [0, 2, 1]
                        else:
                            front = [2, 1, 0]
                    else:
                        front = [2, 1, 0]
                    self.movement.sideways(float(i[1]), front)  # 2,1,0
                    self.movement.set_bearing(self.position_finder)
                elif i[0] == "print":
                    print(i[1])
                elif i[0] == '//':
                    print('[WARN] Commented instruction on line ' +
                          str(route.index(i)+1) + ' -- skipping')
            except Exception as e:
                print("[ERROR] Encountered error running command {} (index {}), error: {}".format(
                    i, self.current_command_index, e))
            self.current_command_index += 1
