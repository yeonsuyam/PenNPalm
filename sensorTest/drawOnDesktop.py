import serial

port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200

mouse = Controller()

def init():
    ser = serial.Serial(port = port, baudrate = baudrate)
    if not ser.is_open:
        print("failed to open the serial port")
        exit()

    # empty the input buffers.
    ser.flushInput()
    ser.flush()

    # throw away some number of data
    for i in range(5):
        x = ser.readline()

    return ser


def main():
	arduino = init()

	while True:
		try:
			line = arduino.readline().decode('ascii')
			dx, dy = tuple(int(num) for num in line.split(', '))

			print("dx: ", dx, "dy: ", dy)


if __name__ == "__main__":
	main()
