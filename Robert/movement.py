import time, math
from sr.robot3 import *
class MovementMaster():
	def __init__(self, robot):
		''' Robot dimension constants'''
		R = robot
		self.arm_radius = 0.25
		self.circumference = 0.3299
		self.motors = [R.motor_boards['SR0PJ1W'].motors[0],R.motor_boards['SR0PJ1W'].motors[1],R.motor_boards['SR0VG1M'].motors[0]]# m0 and m1 are front motors
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
	def forwards(self, distance, power):
		'''function to move the robot forwards and backwards'''
		self.negative = 1
		if distance < 0 :
			self.negative = -1
			distance *= -1
		self.speed = self.circumference * ((4.6778*(power**6) - 88.46*(power**5) - 9.1788*(power**4) + 82.827*(power**3) + 5.6922*(power**2) + 97.405*(power))/60)
		self.time_to_move = distance/self.speed
		self.motors[0].power = power * -1 * self.negative
		self.motors[1].power = power * self.negative
		time.sleep(self.time_to_move)
		self.motors[0].power = BRAKE
		self.motors[1].power = BRAKE
