#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def gps_callback(data):
    print("Latitude: {}".format(data.latitude))
    print("Longitude: {}".format(data.longitude))
    print("Altitude: {}".format(data.altitude))

def listener():
    rospy.init_node('gps_listener', anonymous=True)
    rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
