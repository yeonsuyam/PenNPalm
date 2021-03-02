import sys
import argparse
import serial
import time
from datetime import datetime
import numpy as np
from numpy import power, sqrt, sin, cos
import matplotlib.pyplot as plt
import matplotlib
from pynput.mouse import Button, Controller


class Mouse():
	def __init__(self, mouse, movingAverageWeight, d, x, y, theta, pitchStandard, rollStandard):
		self.mouse = mouse
		self.weight = movingAverageWeight
		
		self.d = d
		self.x = x
		self.y = y
		self.theta = theta
		self.pitchStandard = pitchStandard
		self.rollStandard = rollStandard

		self.dx = 0
		self.dy = 0

		return
	

	def getExponentialMovingAverage(self, dx, dy):
		# print(dx, dy)
		self.dx = self.dx * (1-self.weight) + dx * self.weight
		self.dy = self.dy * (1-self.weight) + dy * self.weight

		return self.dx, self.dy


	def pitchrollTodxdy(self, pitch, roll):
		# Relative
		dx = self.d * sin((pitch - self.pitchStandard) / 180 * np.pi)
		dy = self.d * sin((roll - self.rollStandard) / 180 * np.pi)

		dx, dy = self.getExponentialMovingAverage(dx, dy)

		# Rotation matrix
		dxRotate = dx * cos(self.theta) - dy * sin(self.theta)
		dyRotate = dx * sin(self.theta) + dy * cos(self.theta)

		return dxRotate, dyRotate


	def mouseMove(self, pitch, roll):
		dx, dy = self.pitchrollTodxdy(pitch, roll)
		self.mouse.position = (self.x + dx, self.y - dy)
