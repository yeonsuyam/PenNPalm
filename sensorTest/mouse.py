import sys
import argparse
import serial
import time
from datetime import datetime
import numpy as np
from numpy import power, sqrt, sin, cos, tan
import matplotlib.pyplot as plt
import matplotlib
from pynput.mouse import Button, Controller
from OneEuroFilter import OneEuroFilter
import datetime


class Mouse():
	def __init__(self, mouse, movingAverageWeight, d, x, y, theta, pitchStandard, rollStandard):
		self.mouse = mouse
		self.weight = movingAverageWeight
		
		self.d = d
		self.x = x
		self.y = y
		self.mouse.position = (x, y)

		self.theta = theta
		self.pitchStandard = pitchStandard
		self.rollStandard = rollStandard

		self.dx = 0
		self.dy = 0

		# self.initOneEuroFilter(3.0, 0.005)

		return


	def initOneEuroFilter(self, beta, cutoff):
		self.config = {
			'freq': 100,       # Hz
			'mincutoff': 0.0005,  # FIXME
			'beta': 3.0,       # FIXME
			'dcutoff': 0.0005     # this one should be ok
		}

		self.oneEuroFilterX = OneEuroFilter(**self.config)
		self.oneEuroFilterY = OneEuroFilter(**self.config)
		self.timestamp = 0
		self.time = datetime.datetime.now()

		return


	def getExponentialMovingAverage(self, dx, dy):
		# print(dx, dy)
		self.dx = self.dx * (1-self.weight) + dx * self.weight
		self.dy = self.dy * (1-self.weight) + dy * self.weight

		return self.dx, self.dy


	def getOneEuroFilter(self, dx, dy):
		self.timestamp += (datetime.datetime.now() - self.time).microseconds/1000000
		self.dx = self.oneEuroFilterX(dx, self.timestamp)
		self.dy = self.oneEuroFilterY(dy, self.timestamp)
		self.time = datetime.datetime.now()
		# print((datetime.datetime.now() - self.time).microseconds)
		return self.dx, self.dy


	def pitchrollTodxdy(self, pitch, roll):
		# Relative
		dx = self.d * tan((pitch - self.pitchStandard) / 180 * np.pi)
		dy = self.d * tan((roll - self.rollStandard) / 180 * np.pi)

		# dx, dy = self.getExponentialMovingAverage(dx, dy)
		dx, dy = self.getOneEuroFilter(dx, dy)

		# Rotation matrix
		dxRotate = dx * cos(self.theta) - dy * sin(self.theta)
		dyRotate = dx * sin(self.theta) + dy * cos(self.theta)

		return dxRotate, dyRotate


	def mouseMove(self, pitch, roll):
		dx, dy = self.pitchrollTodxdy(pitch, roll)
		self.mouse.position = (self.x + dx, self.y - dy)

	def drawMove(self, dx, dy):
		self.mouse.move(dx, -dy)

	def press(self):
		self.mouse.press(Button.left)

	def release(self):
		self.mouse.release(Button.left)

	def setPitchRollStandard(self, pitchStandard, rollStandard):
		self.pitchStandard = pitchStandard
		self.rollStandard = rollStandard

	def resetXY(self):
		self.x, self.y = self.mouse.position