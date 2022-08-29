import serial
import time

arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

while True:
    # try:
        # print("try")
        data = arduino.readline()
        # print("data ", data.decode('utf-8'))
        if data:
            print(data.decode('utf-8'))
            # # print("Hi arduino")
            # arduino.write("from nano to arduino")
    # except:
    #     print("catch")
    #     arduino.close()