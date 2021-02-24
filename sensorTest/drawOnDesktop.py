import serial
from pynput.mouse import Button, Controller
from multiprocessing import Process, Queue
import queue

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


def main(xData, yData):
    arduino = init()

    while True:
        try:
            line = arduino.readline().decode('ascii')
            
            dx, dy = tuple(int(num) for num in line.split(', '))

            # print("dx: ", dx, "dy: ", dy)

            xData.put(dx)
            yData.put(dy)

        except:
            pass


def isTouch(data):
    if data >= 999:
        return False
    else:
        return True


def mouseDraw(xData, yData):
    isDrawFlag = 2

    while True:
        try:
            dx, dy = xData.get(False), yData.get(False)
            
        except queue.Empty:
            pass
        
        else:
            if not isTouch(dx):
                mouse.release(Button.left)
                mouse.move(50, 0)
                isDrawFlag = 2

            else:
                if isDrawFlag > 0:
                    isDrawFlag -= 1
                    if isDrawFlag == 0:
                        mouse.press(Button.left)

                mouse.move(dx, -dy)


if __name__ == "__main__":
    xData, yData = Queue(), Queue()
    mainProcess = Process(target=main, args=(xData, yData,), daemon=True)
    mainProcess.start()

    mouseDraw(xData, yData)
