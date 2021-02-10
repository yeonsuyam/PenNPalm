import serial
import time
from datetime import datetime
import numpy as np
from numpy import power, sqrt, sin, cos
import matplotlib.pyplot as plt
import matplotlib

port = "COM4"
baudrate = 115200
PRINT_SPEED = 1


def init():
    ser = serial.Serial(port = port, baudrate = baudrate)
    if not ser.is_open:
        print("failed to open the serial port")
        exit()

    # empty the input buffers.
    ser.flushInput()
    ser.flush()

    # throw away some number of packets. they are not in sync. i dont know why.
    for i in range(10):
        x = ser.readline()

    return ser


def visualize(ax, pitch, roll):
	p = -pitch
	r = roll

	print(p * 180 / np.pi, r * 180 / np.pi)
	# Prepare arrays x, y, z
	zLength = cos(r) * cos(p)

	z = np.linspace(-zLength, 0, 50)
	x = (-1) * z * sin(p) / cos(p)
	y = (-1) * z * sin(r) / cos(r) / cos(p)

	# print("x: ", x[0], "y: ", y[0], "z: ", z[0])

	ax.plot(x, y, z, label='parametric curve')
	plt.pause(PRINT_SPEED)


def main():
	arduino = init()

	plt.rcParams['legend.fontsize'] = 10

	fig = plt.figure()
	ax = fig.gca(projection='3d')
	ax.set_xlim(-1, 1)
	ax.set_ylim(1, -1)
	ax.set_zlim(-1, 0)

	for i in range(10000):
		line = arduino.readline().decode('ascii')
		pitch, roll = tuple(line.split(', '))

		visualize(ax, float(pitch), float(roll))
	

if __name__ == "__main__":
    matplotlib.rcParams['axes.formatter.useoffset'] = False

    main()
    # test()

    plt.show()