import time, math
from sr.robot3 import *

class MovementMaster():
	def __init__(self, robot):
		''' Robot dimension constants'''
		R = robot
		self.arm_radius = 0.25
		self.circumference = 0.3299
		self.motors = [R.motor_boards['SR0PJ1W'].motors[0],R.motor_boards['SR0PJ1W'].motors[1],R.motor_boards['SR0VG1M'].motors[0]]# m0 and m1 are front motors
		self.intergrated_distance = 0.199569041087827
		self.rps_constant = 1.54090799872
	def rotate(self, angle, power):
		''' function to turn the robot'''
		self.negative = 1
		if angle < 0:
			self.negative = -1
			angle *= -1
		self.speed = self.circumference * ((4.6778*(power**6) - 88.46*(power**5) - 9.1788*(power**4) + 82.827*(power**3) + 5.6922*(power**2) + 97.405*(power))/60)
		self.distance = angle * ((math.pi * 2 * self.arm_radius) / 360)
		self.time_to_move = self.distance/self.speed
		for wheel in self.motors:
			wheel.power = power*self.negative
		time.sleep(self.time_to_move)
		for wheel in self.motors:
			wheel.power = BRAKE
	def forwards(self, distance, front_wheels = [0,1]):
		'''function to move the robot forwards and backwards
		Distance: the distance you wish to move the robot
		front_wheels: the index of the wheels you wish to use (normally 0 and 1)'''
		self.negative = 1
		if distance < 0 :
			self.negative = -1
			distance *= -1
		if distance > self.intergrated_distance:
			distance = self.accelerate_forwards(distance, negative
		power = 0.8
		self.speed = self.circumference * self.rps_constant
		self.time_to_move = distance/self.speed
		self.motors[front_wheels[0]].power = 0.8 * -1 * self.negative
		self.motors[front_wheels[1]].power = 0.8 * self.negative
		time.sleep(self.time_to_move)
		self.motors[0].power = BRAKE
		self.motors[1].power = BRAKE


	def accelerate_forwards(self, distance, negative):
		''' The function to make the motors accelerate the robot
		Distance: the distance for the wheels to go
		Negative: to tell us to reverse motors'''
		num_of_changes = 8
		time_to_accelerate = 0.5
		time_per_step = time_to_accelerate/num_of_changes

		for i in range(num_of_changes):
			self.motors[front_wheels[0]].power = (0.8/num_of_changes)*i * -1 * negative
			self.motors[front_wheels[1]].power = (0.8/num_of_changes)*i * negative
			time.sleep(time_per_step)

		return distance-self.intergrated_distance