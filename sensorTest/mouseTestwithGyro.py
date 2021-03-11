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
from mouse import Mouse

# sys.stdout = open('output.txt','w')

# port = "COM4"
port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200



# def parse_args():
# 	parser = argparse.ArgumentParser()

# 	parser.add_argument("--name", dest="data_name", type=str, required=True, help="name of measuring environment. ex) cylinder, hongsam, ..")
# 	parser.add_argument("--baseline", dest="baseline_file_name", type=str, required=False, help="name of baseline file")
# 	# parser.add_argument("--data", dest="data_folder_path", type=str, required=True, help="path of data folder")
# 	# No baseline 비슷한 argument 만들어서 baseline들을 그냥 measure함.

# 	return parser.parse_args()


def init():
    ser = serial.Serial(port = port, baudrate = baudrate)
    if not ser.is_open:
        print("failed to open the serial port")
        exit()

    # empty the input buffers.
    ser.flushInput()
    ser.flush()

    # throw away some number of packets. they are not in sync. i dont know why.
    for i in range(20):
        x = ser.readline()

    return ser


def mouseMove(pitch, roll):
	global d, x, y, pitchStandard, rollStandard

	# Relative
	dx = d * sin((pitch - pitchStandard) / 180 * np.pi)
	dy = d * sin((roll - rollStandard) / 180 * np.pi)

	theta = 0
	# Rotation matrix
	dxRotate = dx * cos(theta) - dy * sin(theta)
	dyRotate = dx * sin(theta) + dy * cos(theta)

	dx, dy = exponentialMovingAverage(dx, dy)

	mouse.position = (x + dx, y - dy)


def visualize(axAbsolute, axRelative, pitch, roll):
	p = pitch
	r = -roll

	# Absolute
	zLength = cos(r) * cos(p)
	z = np.linspace(-zLength, 0, 50)
	x = (-1) * z * sin(p) / cos(p)
	y = (-1) * z * sin(r) / cos(r) / cos(p)

	axRelative.plot(x, y, z)

	# Relative
	p = pitch - pitchStandard
	r = -(roll - rollStandard)

	zLength = cos(r) * cos(p)
	z = np.linspace(-zLength, 0, 50)
	x = (-1) * z * sin(p) / cos(p)
	y = (-1) * z * sin(r) / cos(r) / cos(p)

	axAbsolute.plot(x, y, z)


def draw():
	global pitchStandard
	global rollStandard

	plt.rcParams['legend.fontsize'] = 10

	fig = plt.figure()
	axAbsolute = fig.add_subplot(1, 2, 1, projection='3d')
	axRelative = fig.add_subplot(1, 2, 2, projection='3d')
	
	# ax = fig.gca(projection='3d')
	axAbsolute.set_xlim(-1, 1)
	axAbsolute.set_ylim(1, -1)
	axAbsolute.set_zlim(-1, 0)
	axAbsolute.set_title("Absolute")
	axAbsolute.legend()
	axAbsolute.set_xlabel('$X$', fontsize=20, rotation=150)
	axAbsolute.set_ylabel('$Y$', fontsize=20, rotation=150)
	axAbsolute.set_zlabel('$Z$', fontsize=30, rotation=60)


	axRelative.set_xlim(-1, 1)
	axRelative.set_ylim(1, -1)
	axRelative.set_zlim(-1, 0)
	axRelative.set_title("Relative")
	axRelative.legend()
	axRelative.set_xlabel('$X$', fontsize=20, rotation=150)
	axRelative.set_ylabel('$Y$', fontsize=20, rotation=150)
	axRelative.set_zlabel('$Z$', fontsize=30, rotation=60)


	f = open("./output.txt", 'r')
	for i in range(2):
		f.readline()

	pitchStandard, rollStandard = tuple(float(number) for number in f.readline().strip().split(" "))

	print(pitchStandard, rollStandard)

	for i in range(80):
		pitch, roll = tuple(f.readline().strip().split(" "))
		visualize(axAbsolute, axRelative, float(pitch)/180 * np.pi, float(roll)/180 * np.pi)

	f.close()



def main():
	arduino = init()
	mouse = Mouse(Controller(), 0.5, 4000, 1000, 200, 0, 0, 0, usingGyro = True)

	print("init finish")
	

	for i in range(100000):
		line = arduino.readline().decode('ascii')
		pitch, roll = tuple(line.split(', '))

		mouse.mouseMove(int(float(pitch)/8), int(float(roll)/8))


if __name__ == "__main__":
	main()
	plt.show()
