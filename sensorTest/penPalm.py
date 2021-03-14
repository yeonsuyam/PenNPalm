import serial
from multiprocessing import Process, Queue
import queue
from pynput.mouse import Button, Controller
from pynput import keyboard
from mouse import Mouse
import datetime

port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200

defaultMouseGain = 3
smallMouseGain = 10


class PenPalm():
	def __init__(self, dataQueue, debug=False):
		self.dataQueue = dataQueue

		self.mouse = Mouse(Controller(), 0.5, 2000, 1000, 200, 0, 0, 0, True)
		self.mouseGain = defaultMouseGain

		self.capacitiveBoundaryTouch = 100
		self.capacitiveBoundaryHover = 80
		self.capacitiveIsTouchHoverNone = 0 # 0: None, 1: Hover, 2: Touch
		self.previousCapacitiveStates = [0 for i in range(10)]
		self.capacitive = 0

		self.isWritingState = False
		self.isWritingStarted = False

		# For debugging
		self.debug = debug
		self.timestamp = datetime.datetime.now()

	def run(self):
		while True:
			if self.getData():
				if self.isTouch():
					if self.previousCapacitiveStates[-1] != 2:
						if self.displacementX == -0.0 and self.displacementY == -0.0:
							if not self.isWritingState:
								self.startWritngState()
						else:
							print(self.displacementX, self.displacementY)
							self.mouse.press()
							self.isWritingStarted = True
					self.mouse.drawMove(self.displacementX, self.displacementY)

				elif self.isHover():
					self.mouse.release()
					# TODO: only run isClick() function when case 2
					if self.isWritingState and not self.isWritingStarted:
						continue
					else:
						self.mouse.mouseMove(self.gyroX, self.gyroY)
					# TODO: self.mouse.mouseMove(self.displacementX, self.displacementY)

				elif self.isNone():
					self.mouse.release()
					# TODO: only run isClick() function when case 2
					if self.isWritingState and not self.isWritingStarted:
						continue
					else:
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

	def isClick(self):
		if self.isTouch() and self.previousCapacitiveStates[-1] != 2:
			if self.displacementX == 0 and self.displacementY == 0:
				if self.isWritingState:
					self.isWritingState = False
				else:
					self.isWritingState = True
				return True
			else:
				return False

	def startWritngState(self):
		print("Start isWritingState")
		self.isWritingState = True
		self.isWritingStarted = False
		self.mouseGain = smallMouseGain
		return

	def endWritingState(self):
		self.isWritingState = False
		self.isWritingStarted = False
		self.mouseGain = defaultMouseGain
		return

	def calibrateGyro(self):
		self.gyroX = int(self.gyroX / self.mouseGain)
		self.gyroY = int(self.gyroY / self.mouseGain)
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

def on_press(key):
	print("End Writing State")
	penPalm.endWritingState()
	return

if __name__ == "__main__":
	# args = parse_args()
	data = Queue()
	mainProcess = Process(target=main, args=(data, ), daemon=True)
	mainProcess.start()

	listener = keyboard.Listener(
		on_press=on_press)
	listener.start()

	penPalm = PenPalm(data, False)
	penPalm.run()

