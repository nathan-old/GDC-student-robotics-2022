import time, math
from sr.robot3 import *

class MovementMaster():
	def __init__(self, robot):
		''' Robot dimension constants'''
		print('[INITI] Movement Master initialised')
		R = robot
		self.R = robot
		self.arm_radius = 0.25
		self.wheel_circumference = 0.3299
		self.motors = [R.motor_boards['SR0PJ1W'].motors[0], R.motor_boards['SR0VG1M'].motors[0], R.motor_boards['SR0VG1M'].motors[1]]# m0 and m1 are front motors
		self.intergrated_distance_acceleration = 0.199569041087827
		self.rps_constant = 1.54090799872
		self.speed = self.wheel_circumference * self.rps_constant

	def rotate(self, angle, power):
		''' function to turn the robot'''
		print('Turning - ' + str(angle) + ' at power - ' + str(power))
		angle /= 1.13
		negative = 1
		if angle < 0:
			negative = -1
			angle *= -1
		speed = self.wheel_circumference * ((4.6778*(power**6) - 88.46*(power**5) - 9.1788*(power**4) + 82.827*(power**3) + 5.6922*(power**2) + 97.405*(power))/60)
		time_to_move = (angle * ((math.pi * 2 * self.arm_radius) / 360))/speed
		for wheel in self.motors:
			wheel.power = power*negative
		time.sleep(time_to_move)
		for wheel in self.motors:
			wheel.power = BRAKE
		
	def sideways(self, distance, front_wheels = [2,1,0]):
		print('sideways - ' + str(distance))
		distance /= 0.966982143
		negative = 1
		if distance < 0 :
			negative = -1
			distance *= -1
		power = 0.63
		speed = self.wheel_circumference * ((4.6778*(power**6) - 88.46*(power**5) - 9.1788*(power**4) + 82.827*(power**3) + 5.6922*(power**2) + 97.405*(power))/60)
		time_to_move = distance/speed
		self.motors[front_wheels[0]].power = power * negative
		self.motors[front_wheels[1]].power = -0.35 * negative
		self.motors[front_wheels[2]].power = -0.35 * negative
		time.sleep(time_to_move)
		for wheel in self.motors:
			wheel.power = BRAKE


	def forwards(self, distance, front_wheels = [0,1]):
		'''function to move the robot forwards and backwards
		Distance: the distance you wish to move the robot
		front_wheels: the index of the wheels you wish to use (normally 0 and 1)'''
		print('forwards - ' + str(distance))
		distance *= 0.9293
		distance /= 1.11
		self.negative = 1
		if distance < 0 :
			self.negative = -1
			distance *= -1
		if distance > (self.intergrated_distance_acceleration):
			distance = self.accelerate_forwards(distance, self.negative, front_wheels)
			time_to_move = distance/self.speed
			self.motors[front_wheels[0]].power = 0.8 * self.negative
			self.motors[front_wheels[1]].power = 0.8 * -1 * self.negative
			time.sleep(time_to_move)
			#self.decelerate_forwards(self.negative, front_wheels)
			self.motors[front_wheels[0]].power = BRAKE
			self.motors[front_wheels[1]].power = BRAKE
		else:
			time_to_move = distance/(self.wheel_circumference * 0.528070517)
			self.motors[front_wheels[0]].power = 0.3 * self.negative
			self.motors[front_wheels[1]].power = 0.3 * -1 * self.negative
			time.sleep(time_to_move)
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
			self.motors[front_wheels[0]].power = (0.8/num_of_changes)*i* negative
			self.motors[front_wheels[1]].power = (0.8/num_of_changes)*i* -1 * negative
			time.sleep(time_per_step)
			self.motors[front_wheels[1]].power = ((0.8/num_of_changes)*(i+1) * -1 * negative)
			self.motors[front_wheels[0]].power = ((0.8/num_of_changes)*(i+1) * negative)
			time.sleep(time_per_step)

		return distance- self.intergrated_distance_acceleration
	def curve(self, radius, angle, power = 0.3):
		if angle < 0:
			angle *= -1
			negative = -1
		else:
			negative = 1
		speed = self.wheel_circumference * ((4.6778*(power**6) - 88.46*(power**5) - 9.1788*(power**4) + 82.827*(power**3) + 5.6922*(power**2) + 97.405*(power))/60)
		distance = math.pi * (2 *radius) * (float(angle)/360.0)
		print(distance, speed, angle)
		time_to_drive = distance / speed
		rotatanal_speed = (angle * ((math.pi * 2 * self.arm_radius) / 360))/time_to_drive
		rotatanal_speed *= negative
		print(speed + rotatanal_speed, (speed * -1) + rotatanal_speed, rotatanal_speed)
		self.motors[0].power = speed + rotatanal_speed
		self.motors[1].power = (speed * -1) + rotatanal_speed
		self.motors[2].power = rotatanal_speed
		time.sleep(time_to_drive)
		self.motors[0].power = BRAKE
		self.motors[1].power = BRAKE
		self.motors[2].power = BRAKE
	def pos_get(self, position_finder):
		position = position_finder.try_untill_find()
		if position != None:
			print('[INFO] You are in position ('+str(round(position[0][0], 2))+',' +
				str(round(position[0][1], 2))+') at an angle of '+str(round(position[1], 2)))
			return position
		else:
			print('[WARN] Cannot find a position')
			return None
	def set_bearing(self, pos, tolerance=2, tries=3):
		if int(self.R.zone) == 0:
			start_beraing = 105 # 165
		elif int(self.R.zone) == 1:
			start_beraing = 195 # 255
		elif int(self.R.zone) == 2:
			start_beraing = 285 # 345
		elif int(self.R.zone) == 3:
			start_beraing = 15 # 075
		for i in range(tries):
			position = self.pos_get(pos)
			if position is None:
				continue

			turn_angle = float(position[1]) - float(start_beraing)
			self.rotate(turn_angle, 0.3)
			break

class RouteCommands():
	def __init__(self, R, movement, Grabber_Enable, com, position_finder):
		self.movement = movement
		self.Grabber_Enable = Grabber_Enable
		self.com = com
		self.position_finder = position_finder
		self.R = R
		print('[INITI] Route follower initialised')
	def follow(self, route):
		for i in route:
			if len(i) < 2:
				print('[WARN] Invalid instruction on line ' + str(route.index(i)+1) + ' -- skipping')
				continue
			if i[0] == 'forwards':
				if len(i) == 3:
					if i[2] == 'Plough\n' or i[2] == 'Plough':
						front = [2, 0]
					elif i[2] == 'Empty\n' or i[2] == 'Empty':
						front = [1, 2]
					else:
						front = [0,1]
				else:
					front = [0,1]
				self.movement.forwards(float(i[1]), front)# [2,0]
			elif i[0] == 'beep':
				self.R.power_board.piezo.buzz(float(i[1]), Note.D6)
			elif i[0] == 'turn':
				self.movement.rotate(float(i[1]), 0.3)
			elif i[0] == 'sleep':
				time.sleep(float(i[1]))
			elif i[0] == 'grab' and self.Grabber_Enable:
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
					front = [2,1,0]
				self.movement.sideways(float(i[1]), front)#2,1,0
			elif i[0] == 'bearing':
				self.movement.set_bearing(self.position_finder)
			elif i[0]=='//':
				print('[WARN] Commented instruction on line ' + str(route.index(i)+1) + ' -- skipping')
			