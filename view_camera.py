#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
# cv2 is not strictly needed if we don't display the window
# import cv2 

# This function is the "callback". It runs EVERY time a message is received.
def image_callback(msg):
    """
    This function is called every time a new image is published on the topic.
    """
    # We use rospy.loginfo to print messages. It's the standard ROS way.
    rospy.loginfo("SUCCESS: An image frame was received!")

    # The code below would convert the image for use with OpenCV
    # but for the demo, just proving we received the message is enough.
    # bridge = CvBridge()
    # try:
    #     cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    #     cv2.imshow("Camera Feed", cv_image)
    #     cv2.waitKey(1)
    # except Exception as e:
    #     rospy.logerr(e)


def main():
    # 1. Initialize the node.
    rospy.init_node('camera_viewer_node')
    rospy.loginfo("Camera viewer node has been started.")

    # 2. Set the correct topic name. THIS IS THE KEY FIX.
    # The name comes from your 'rostopic list' output.
    camera_topic = "/head_camera/depth_downsample/image_raw"

    # 3. Subscribe to the topic.
    # ROS will now listen for messages on this topic and call 'image_callback' when one arrives.
    rospy.Subscriber(camera_topic, Image, image_callback)
    rospy.loginfo("Subscribed to topic: %s", camera_topic)
    rospy.loginfo("Waiting for image frames... Make sure the simulation is running.")

    # 4. rospy.spin() keeps the script from exiting.
    # It waits until you shut it down with Ctrl+C.
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass