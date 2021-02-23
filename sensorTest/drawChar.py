import serial
from PIL import Image, ImageColor
import os
# from scipy.interpolate import spline
import os
import time
from datetime import datetime
import cv2
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue
import argparse
import numpy as np
import queue



size = 500
port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200
PRINT_SPEED = 0.8

NEXTCHAR = 1
NEXTWORD = 2
POINTING = 3

width, height = 50, 128

def visualize(x, y):
	x, y = 50, 128
	
	xList = []
	yList = []

	img = 255 * np.ones(shape=[height * 2, width*10, 3], dtype=np.uint8)
	# cv2.imshow("White Blank", img)

	numChar = 0
	flag = 0

	while True:
		try:
			dx, dy = xData.get(False), yData.get(False)
			
		except queue.Empty:
			pass
		
		else:
			if dx >= 999:
				numChar += 1
				flag += 1
				continue

			else:
				if flag == NEXTCHAR:
					x += numChar * 70
					y = 128
					numChar = 0

				elif flag == NEXTWORD:
					# reset canvas
					img = 255 * np.ones(shape=[height * 2, width*10, 3], dtype=np.uint8)
					x = width
					y = height

				elif flag == POINTING:
					# reset canvas
					img = 255 * np.ones(shape=[height * 2, width*10, 3], dtype=np.uint8)
					x = width
					y = height
					print("TODO: Start Pointing")

				img = cv2.line(img, (x, y), (x+dx, y - dy), (0, 0, 0), 1)
				x += dx
				y -= dy
				cv2.imshow("White Blank", img)
				
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break

				flag = 0


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

	while True:
		try:
			line = arduino.readline().decode('ascii')
			
			dx, dy = tuple(int(num) for num in line.split(', '))

			print("dx: ", dx, "dy: ", dy)

			xData.put(dx)
			yData.put(dy)

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
	
