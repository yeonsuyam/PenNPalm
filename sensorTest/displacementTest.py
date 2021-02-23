import serial
from PIL import Image, ImageColor
import os
# from scipy.interpolate import spline
import os
import time
from datetime import datetime
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue
import argparse
import numpy as np
import queue



size = 500
port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200
PRINT_SPEED = 0.8


def visualize(x, y):
	xList = []
	yList = []
	length = 2000

	plt.figure(figsize=(5, 5))
	plt.cla()
	plt.xlim(-500,500)
	plt.ylim(-500,500)

	plt.plot(np.array([0]), np.array([0]))

	plt.pause(0.1)

	cnt = 0

	while True:
		try:
			x, y = xData.get(False), yData.get(False)
			
		except queue.Empty:
			pass
		
		else:
			if len(xList) > length:
				xList.pop(0)
				yList.pop(0)

			xList.append(x)
			yList.append(y)

			cnt += 1

			if cnt % 50 == 0:
				# cnt = 0
				plt.cla()
				plt.xlim(-500, 500)
				plt.ylim(-500, 500)

				plt.plot(np.array(xList), np.array(yList))

				plt.pause(0.001)


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


def main(xData, yData):
	arduino = init()

	x, y = 0, 0
	while True:
		try:
			line = arduino.readline().decode('ascii')
			
			dx, dy = tuple(int(num) for num in line.split(', '))

			if dx == 999:
				continue

			# print("dx: ", dx, "dy: ", dy)

			x += dx
			y += dy

			xData.put(x)
			yData.put(y)

		except:
			pass

	im.save('simplePixel.png') # or any image format
	os.system("open simplePixel.png") #Will open in Preview.

if __name__ == "__main__":
	# arduino = init()

	xData, yData = Queue(), Queue()
	mainProcess = Process(target=main, args=(xData, yData,), daemon=True)
	mainProcess.start()

	visualize(xData, yData)
	
