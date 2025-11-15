#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    """
    This function is called every time a new Odometry message is received.
    """
    # Get the position from the message
    position = msg.pose.pose.position
    
    # Log the robot's X and Y position to the screen
    rospy.loginfo("Robot is at position: [x: %.2f, y: %.2f]", position.x, position.y)


def main():
    rospy.init_node('odometry_checker_node')
    rospy.loginfo("Odometry checker node started.")

    # The /odom topic is guaranteed to have data if the robot is moving
    odom_topic = "/odom" 

    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.loginfo("Subscribed to topic: %s", odom_topic)

    rospy.spin()

if __name__ == '__main__':
    main()