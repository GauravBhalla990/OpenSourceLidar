#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from hello import gather_distance_data
import serial

arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def init_port():
    serialPortName = '/dev/ttyUSB0'
    serialPortBaudRate = 115200
    
    port = serial.Serial(serialPortName, serialPortBaudRate)#, timeout=0.1)
    
    return port

def get_dist(port):
    distanceStr = port.readline()
    
    # Convert the string to a numeric distance value.
    try:
        # print(distanceStr.decode())
        splitStr = distanceStr.decode().split(" ")
        distance = float(splitStr[0])
    except ValueError:
        # It is possible that the SF30 does not get a valid signal, we represent this case as a -1.0m.
        distance = -1.0    
    
    return distance
rospy.init_node('laser_scan_publisher')

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)

num_readings = 60 #100
laser_frequency =  625 # 20000 (5000 RPM motor = 0.012 seconds / revolution)

count = 0
r = rospy.Rate(1.0)

port = init_port()

while not rospy.is_shutdown():
    speed = arduino.readline()
    if speed.decode('utf-8'):
        print("speed.decode('utf-8') ", speed.decode('utf-8'))
        num_readings =int(laser_frequency/float(speed.decode('utf-8')))
            
    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = -3.14 #-1.57
    scan.angle_max = 3.14 #1.57
    scan.angle_increment = 2*3.14 / num_readings
    scan.time_increment = (1.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 100.0

    scan.ranges = []
    scan.intensities = []

    

    for i in range(0, num_readings):
        dist = get_dist(port)
        # print(dist,i)
        scan.ranges.append(1.0 * dist)  # fake data
        # 
        # print(dist, count)
        # print(1.0 * count)s
        # scan.ranges.append(dist)
        scan.intensities.append(1)  # fake data

    scan_pub.publish(scan)
    count += 1
    # r.sleep()
