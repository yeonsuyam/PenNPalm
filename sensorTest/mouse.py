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

# sys.stdout = open('output.txt','w')

# port = "COM4"
port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200

mouse = Controller()

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
    for i in range(5):
        x = ser.readline()

    return ser


def mouseMove(pitch, roll):
	global d, x, y, pitchStandard, rollStandard
	print((pitch - pitchStandard)/np.pi * (-180), (roll - rollStandard)/np.pi * (-180))

	# Relative
	dx = d * sin(pitch - pitchStandard)
	dy = d * sin(roll - rollStandard)

	theta = 0
	# Rotation matrix
	dxRotate = dx * cos(theta) - dy * sin(theta)
	dyRotate = dx * sin(theta) + dy * cos(theta)

	mouse.position = (x + dx, y + dy)


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

	global pitchStandard
	global rollStandard

	print("Initialization for relative plotting")
	pitch, roll = 0.0, 0.0

	for i in range(10):
		line = arduino.readline().decode('ascii')
		pitch, roll = tuple(map(sum, zip((pitch, roll), tuple(float(number) for number in line.split(', ')))))

	pitchStandard, rollStandard = pitch / 10, roll / 10

	print("Finished: pitch: ", pitchStandard, "roll: ", rollStandard)	


	for i in range(100000):
		line = arduino.readline().decode('ascii')
		pitch, roll = tuple(line.split(', '))

		mouseMove(float(pitch), float(roll))




if __name__ == "__main__":
	d = 1000
	x, y = 600, 200
	
	# args = parse_args()
	# main(args)
	main()
	# draw()

	plt.show()