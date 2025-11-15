#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_in_circle():
    # Initialize the ROS node
    rospy.init_node('robot_driver', anonymous=True)
    
    # Create a publisher to send velocity commands
    # velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    velocity_publisher = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)

    # Set the rate at which to publish commands (10 Hz)
    rate = rospy.Rate(10)
    
    rospy.loginfo("Robot is starting to move in a circle")
    
    # Loop until the node is shut down (e.g., by Ctrl+C)
    while not rospy.is_shutdown():
        # Create a Twist message
        vel_msg = Twist()
        
        # Set linear and angular velocities
        vel_msg.linear.x = 0.2  # Move forward
        vel_msg.angular.z = 0.4 # Turn
        
        # Publish the message
        velocity_publisher.publish(vel_msg)
        
        # Wait for the next cycle
        rate.sleep()

if __name__ == '__main__':
    try:
        move_in_circle()
    except rospy.ROSInterruptException:
        pass