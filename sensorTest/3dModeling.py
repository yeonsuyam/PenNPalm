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
import datetime
from visualization import Simulation

port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200

def init():
    ser = serial.Serial(port = port, baudrate = baudrate)
    if not ser.is_open:
        print("failed to open the serial port")
        exit()

    # empty the input buffers.
    ser.flushInput()
    ser.flush()

    # throw away some number of packets. they are not in sync. i dont know why.
    for i in range(30):
        x = ser.readline()

    return ser

def main(data):

	arduino = init()

	while True:
		try:
			line = arduino.readline().decode('ascii')
			# roll, pitch, yaw = tuple(float(num) for num in line.split(', '))
			# rollData.put(roll)
			# pitchData.put(pitch)
			# yawData.put(yaw)

			q1, q2, q3, q4 = tuple(float(num) for num in line.split(', '))
			data.put((q1, q2, q3, q4))

		except:
			pass

if __name__ == "__main__":
	# arduino = init()

	# rollData, pitchData, yawData = Queue(), Queue(), Queue()
	# mainProcess = Process(target=main, args=(rollData, pitchData, yawData, ), daemon=True)
	# mainProcess.start()

	# sim = Simulation(rollData, pitchData, yawData)

	data = Queue()
	mainProcess = Process(target=main, args=(data, ), daemon=True)
	mainProcess.start()

	sim = Simulation(data)
	
	sim.run()
	