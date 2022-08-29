from asyncio import gather
import serial
 
print('Running SF30 sample.')

def init_serial():
     # Make a connection to the com port. USB0 is the first default port assigned to USB serial devices.
    serialPortName = '/dev/ttyUSB0'
    serialPortBaudRate = 115200
    
    port = serial.Serial(serialPortName, serialPortBaudRate)#, timeout=0.1)
    
    # Clear buffer of any partial responses.
    port.readline()
    
def gather_distance_data():
    # Make a connection to the com port. USB0 is the first default port assigned to USB serial devices.
    serialPortName = '/dev/ttyUSB0'
    serialPortBaudRate = 115200
    
    port = serial.Serial(serialPortName, serialPortBaudRate)#, timeout=0.1)
    
    # Clear buffer of any partial responses.
    # port.readline()
    
    # Continuously gather distance data.
    while True:
        # Each reading is contained on a single line.
        distanceStr = port.readline()
    
        # Convert the string to a numeric distance value.
        try:
            # print(distanceStr.decode())
            splitStr = distanceStr.decode().split(" ")
            distance = float(splitStr[0])
            # print(distance)
        except ValueError:
            # It is possible that the SF30 does not get a valid signal, we represent this case as a -1.0m.
            distance = -1.0    
    
        # Do what you want with the distance information here.
        return distance
