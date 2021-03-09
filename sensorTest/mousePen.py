import serial
from pynput.mouse import Button, Controller
from multiprocessing import Process, Queue
import queue
from mouse import Mouse

import argparse


port = "/dev/cu.usbserial-AK08KN48"
baudrate = 115200

global mouse
mouse = Mouse(Controller(), 0.5, 2000, 1000, 200, 0, 0, 0)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", dest="case", type=int, required=True, help="mouse-pen case")
    parser.add_argument("--filter", dest="filter", type=str, default="default", help="mouse-pen case")
    return parser.parse_args()


def init():
    ser = serial.Serial(port = port, baudrate = baudrate)
    if not ser.is_open:
        print("failed to open the serial port")
        exit()

    # empty the input buffers.
    ser.flushInput()
    ser.flush()

    # throw away some number of data
    for i in range(200):
        x = ser.readline()

    return ser


def main(xData, yData):
    global mouse

    arduino = init()
    print("main")

    while True:
        try:
            line = arduino.readline().decode('ascii')
            
            dx, dy = tuple(float(num) for num in line.split(', '))

            xData.put(dx)
            yData.put(dy)

        except:
            pass


def isDraw(data, flag, case):
    global mouse
    if data == -999:
        print("mouse start")
        mouse.release()
        return False
    if data == 999:
        print("draw start")
        mouse.press()
        return True
    else:
        return flag


def mouseDraw(xData, yData, args):
    global mouse
    isDrawFlag = False

    initcnt = 30
    num = initcnt

    print("mousedraw")

    if args.filter == "strong":
        mouse.initOneEuroFilter(3.0, 0.001)
    elif args.filter == "weak":
        mouse.initOneEuroFilter(2.0, 0.01)
    else:
        mouse.initOneEuroFilter(3.0, 0.005)

    while True:
        try:
            dx, dy = xData.get(False), yData.get(False)
            
        except queue.Empty:
            pass
        
        else:
            if dx == 999 or dx == -999 or dy == 999 or dy == -999:
                isDrawFlag = isDraw(dx, isDrawFlag, args.case)

                if args.case == 2 and isDrawFlag == False: 
                    initcnt = 8
                    num = initcnt
            else:
                if initcnt > 0:
                    if dx == -999 or dy == -999:
                        continue

                    if initcnt == num:
                        print("Initialization for relative plotting")
                        pitch, roll = 0.0, 0.0
                    print(initcnt)
                    pitch += dx
                    roll += dy

                    pitchStandard, rollStandard = pitch / num, roll / num
                    initcnt -= 1

                elif initcnt == 0:
                    print("Finished: pitch: ", pitchStandard, "roll: ", rollStandard)
                    mouse.setPitchRollStandard(pitchStandard, rollStandard)
                    mouse.resetXY()
                    initcnt -= 1

                else: 
                    # print(dx, dy)
                    if isDrawFlag:
                        mouse.drawMove(int(dx), int(dy))

                    elif not isDrawFlag:
                        pitch = dx
                        roll = dy
                        mouse.mouseMove(float(pitch), float(roll))
                    


if __name__ == "__main__":
    args = parse_args()
    xData, yData = Queue(), Queue()
    mainProcess = Process(target=main, args=(xData, yData,), daemon=True)
    mainProcess.start()

    mouseDraw(xData, yData, args)
