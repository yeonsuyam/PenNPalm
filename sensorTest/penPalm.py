import serial
from multiprocessing import Process, Queue
import queue
from pynput.mouse import Button, Controller
from mouse import Mouse
import datetime

port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200

class PenPalm():
	def __init__(self, dataQueue, debug=False):
		self.dataQueue = dataQueue

		self.mouse = Mouse(Controller(), 0.5, 2000, 1000, 200, 0, 0, 0, True)
		self.capacitiveBoundaryTouch = 100
		self.capacitiveBoundaryHover = 80
		self.capacitiveIsTouchHoverNone = 0 # 0: None, 1: Hover, 2: Touch
		self.previousCapacitiveStates = [0 for i in range(10)]
		self.capacitive = 0

		# For debugging
		self.debug = debug
		self.timestamp = datetime.datetime.now()

	def run(self):
		while True:
			if self.getData():
				if self.isTouch():
					if self.previousCapacitiveStates[-1] != 2: 
						self.mouse.press()
					self.mouse.drawMove(self.displacementX, self.displacementY)

				elif self.isHover():
					# if self.previousCapacitiveStates[-1] == 2:
					self.mouse.release()
					self.mouse.mouseMove(self.gyroX, self.gyroY)

				elif self.isNone():
					# if self.previousCapacitiveStates[-1] == 2:
					self.mouse.release()
					self.mouse.mouseMove(self.gyroX, self.gyroY)

				else:
					assert 0, 'Unknown touch state'

	def getData(self):
		try:
			self.capacitive, self.displacementX, self.displacementY, self.gyroX, self.gyroY = tuple(self.dataQueue.get(False))
		except queue.Empty:
			return False
		else:
			if self.debug == True:
				currentTime = datetime.datetime.now()
				# print("Data rate: ", 1/(currentTime - self.timestamp).microseconds/1000000)
				self.timestamp = currentTime

			self.dataFormatCheck()
			self.calibrateGyro()
			self.savePreviousCapacitiveState()

			if self.capacitive < self.capacitiveBoundaryHover:
				self.capacitiveIsTouchHoverNone = 0
			elif self.capacitive < self.capacitiveBoundaryTouch:
				self.capacitiveIsTouchHoverNone = 1
			else:
				self.capacitiveIsTouchHoverNone = 2
			return True

	def isTouch(self):
		if self.capacitiveIsTouchHoverNone == 2:
			if self.debug:
				print("isTouch")
			return True
		else:
			return False

	def isHover(self):
		if self.capacitiveIsTouchHoverNone == 1:
			if self.debug:
				print("isHover")
			return True
		else:
			return False

	def isNone(self):
		if self.capacitiveIsTouchHoverNone == 0:
			if self.debug:
				print("isNone")
			return True
		else:
			return False

	def calibrateGyro(self):
		self.gyroX = int(self.gyroX / 3)
		self.gyroY = int(self.gyroY / 3)
		return

	def dataFormatCheck(self):
		# TODO: Check wheter values are correct. capacitive and displacementXY should be int, gyroXY should be float
		return

	def savePreviousCapacitiveState(self):
		self.previousCapacitiveStates = self.previousCapacitiveStates[1:] + [self.capacitive]
		return

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

def main(data):
    global mouse

    arduino = init()
    print("main")

    while True:
        try:
            line = arduino.readline().decode('ascii')
            data.put([float(num) for num in line.split(', ')])

        except:
            pass

if __name__ == "__main__":
    # args = parse_args()
    data = Queue()
    mainProcess = Process(target=main, args=(data, ), daemon=True)
    mainProcess.start()

    penPalm = PenPalm(data, True)
    penPalm.run()

