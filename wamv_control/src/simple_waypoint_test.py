#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

# Desired waypoint coordinates
waypoint_lat = -33.722414
waypoint_long = 150.673977
def publish_waypoint():
    # Initialize the ROS node
    rospy.init_node('waypoint_publisher', anonymous=True)
    
    # Create a publisher for the waypoint topic
    waypoint_pub = rospy.Publisher('/wamv/waypoint', Point, queue_size=10)
    
    # Define the rate at which to publish the waypoint
    rate = rospy.Rate(20)  # 1 Hz

    # Create the waypoint message
    waypoint_msg = Point()
    waypoint_msg.x = waypoint_lat
    waypoint_msg.y = waypoint_long
    waypoint_msg.z = 0  # Assuming z is 0 for a 2D waypoint

    while not rospy.is_shutdown():
        # Publish the waypoint
        waypoint_pub.publish(waypoint_msg)
        
        # Log the published waypoint
        rospy.loginfo("Published waypoint: (x: %f, y: %f, z: %f)", waypoint_msg.x, waypoint_msg.y, waypoint_msg.z)
        
        # Sleep for the specified loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_waypoint()
    except rospy.ROSInterruptException:
        pass

