#!/usr/bin/env python
# license removed for brevity
from hello import gather_distance_data
import rospy
from std_msgs.msg import String
from hello import gather_distance_data
from sensor_msgs.msg import LaserScan
import math
import random
import time

last_scan_time=0
def deg2rad(deg):
    return deg*(math.pi/180)
def get_random_angle():
    return random(0,360)

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print(gather_distance_data())
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = get_most_recent_timestamp(rf, sg)    
        scan.angle_min = deg2rad(0)
        scan.angle_max = deg2rad(360)
        scan.angle_increment = deg2rad(1)
        scan.scan_time = time.time() - last_scan_time
        #scan.time_increment = scan.scan_time / 541
        scan.range_min = 0
        scan.range_max = 100.0
        scan.ranges = rf.ranges
        for i in range(180*2):
            if sg.ranges[i] < scan.ranges[90 + i] or scan.ranges[90 + i] == 0:
                scan.ranges[90 + i] = sg.ranges[i]
        #pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('laser_scan_merger',anonymous=True)
        talker()
    except rospy.ROSInterruptException:
        pass





