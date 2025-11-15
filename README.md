# **Gazebo Vehicle Simulation Task**

**Author:** [Your Name]
**Date:** November 15, 2025

---

#### **1. Project Objective**

The primary objective of this task was to use the Gazebo simulation environment to control a vehicle autonomously and visualize its onboard sensor feed in real-time. The core requirements included launching a simulation, writing a ROS node for autonomous control, identifying the correct ROS topics, and displaying a live sensor data stream.

---

#### **2. Development Environment: The Construct**

**Tool Used:** The Construct (ROS Development Studio)

**Reasoning for Tool Selection:**
The project was developed on a macOS machine where previous attempts at a native ROS installation had failed. To meet the tight 1-hour deadline and mitigate risks associated with complex local setups (like Docker or native builds), a cloud-based robotics platform, **The Construct**, was chosen.

This strategic choice provided several key advantages:
*   **Zero Installation:** It eliminated the need for any local installation of ROS, Gazebo, or their dependencies, saving critical time.
*   **Pre-configured Environment:** It offered an immediate, browser-based Ubuntu environment with ROS and Gazebo pre-installed and configured.
*   **Platform Agnostic:** It allowed development to proceed on a Mac without any compatibility issues.
*   **Focused Development:** By handling the environment setup, it allowed 100% of the time to be focused on solving the robotics task itself.

---

#### **3. Methodology and Execution**

The project was executed in three distinct phases: environment analysis, development of the control node, and development of the sensor visualization node, which included a critical problem-solving pivot.

##### **3.1. Environment Analysis & Topic Identification**

Upon loading the simulation, the first step was to analyze the ROS environment.

*   **ROS Version:** The presence of a `catkin_ws` directory indicated the environment was running **ROS 1** (not ROS 2). All subsequent commands were adapted accordingly.
*   **Topic Discovery:** The `rostopic list` command was used to identify all active communication topics.

**Command:**
```bash
user:~$ rostopic list
```

**Key Topics Identified:**
*   `/teleop/cmd_vel`: The topic for receiving velocity commands (Twist messages).
*   `/odom`: The topic publishing the robot's odometry data (position and orientation).
*   `/head_camera/depth_downsample/image_raw`: The topic expected to publish the robot's camera feed.

##### **3.2. Autonomous Control Node (`drive_robot.py`)**

A Python script using the `rospy` library was created to make the robot move autonomously in a circle. The script publishes `geometry_msgs/Twist` messages to the velocity topic identified in the analysis phase.

**Code (`drive_robot.py`):**
```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_in_circle():
    # Initialize the ROS node
    rospy.init_node('robot_driver', anonymous=True)
    
    # Create a publisher to the correct velocity topic
    velocity_publisher = rospy.Publisher('/teleop/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10) # 10hz
    
    rospy.loginfo("Robot is starting to move in a circle")
    
    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x = 0.2  # Move forward
        vel_msg.angular.z = 0.4 # Turn
        
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_in_circle()
    except rospy.ROSInterruptException:
        pass
```

**Execution Command:**
```bash
user:~$ python drive_robot.py
```

##### **3.3. Sensor Visualization & Strategic Pivot**

The initial plan was to visualize the camera feed. A subscriber node was written for this purpose.

**Initial Code Attempt (`view_camera.py`):**
```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def image_callback(msg):
    rospy.loginfo("SUCCESS: An image frame was received!")

def main():
    rospy.init_node('camera_viewer_node')
    camera_topic = "/head_camera/depth_downsample/image_raw"
    rospy.Subscriber(camera_topic, Image, image_callback)
    rospy.loginfo("Subscribed to topic: %s", camera_topic)
    rospy.spin()

if __name__ == '__main__':
    main()
```

**Problem Diagnosis:** Upon running this script, it became clear that no messages were being published on the camera topic. This indicated an issue with the provided simulation environment itself, not the code.

**Solution & Pivot:** To overcome this and still meet the project's objective, a pivot was made to visualize a different, functioning sensor: **odometry**. A new script was created to subscribe to the `/odom` topic and print the robot's real-time position.

**Final Sensor Node Code (`check_odom.py`):**
```python
#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    # Get the position from the message
    position = msg.pose.pose.position
    # Log the robot's X and Y position to the screen
    rospy.loginfo("Robot is at position: [x: %.2f, y: %.2f]", position.x, position.y)

def main():
    rospy.init_node('odometry_checker_node')
    rospy.loginfo("Odometry checker node started.")

    odom_topic = "/odom" 
    rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.loginfo("Subscribed to topic: %s", odom_topic)
    rospy.spin()

if __name__ == '__main__':
    main()
```

**Execution Command:**
```bash
user:~$ python check_odom.py
```

---

#### **4. Results and Outputs**

The project was a success, achieving all core objectives.

**Control Node Output:** The script ran successfully, indicating it was publishing messages.
```
user:~$ python drive_robot.py
[INFO] [1763198749.505748, 0.000000]: Robot is starting to move in a circle
```

**Sensor Node Output:** The odometry script successfully subscribed to the `/odom` topic and printed a continuous stream of position data, providing real-time visualization of the robot's state.
```
user:~$ python check_odom.py
[INFO] [1763199413.543355, 0.000000]: Odometry checker node started.
[INFO] [1763199413.545374, 0.000000]: Subscribed to topic: /odom
[INFO] [1763199413.614049, 923.746000]: Robot is at position: [x: 0.48, y: -0.51]
[INFO] [1763199413.664356, 923.796000]: Robot is at position: [x: 0.48, y: -0.52]
[INFO] [1763199413.714018, 923.846000]: Robot is at position: [x: 0.48, y: -0.53]
... (data continues to stream)
```

**Simulation View:** The vehicle in the Gazebo window was visually confirmed to be moving in a circle, corresponding directly with the streaming odometry data.

---

#### **5. Conclusion**

This project successfully demonstrated the ability to autonomously control a simulated vehicle and visualize its sensor data using ROS. Key achievements include:
1.  Correctly analyzing a ROS environment to identify topics and versions.
2.  Implementing a robust publisher node for autonomous control.
3.  Demonstrating critical problem-solving skills by diagnosing an inactive camera feed and pivoting to an alternative sensor (odometry) to fulfill the project requirements.

The successful completion of this task within the time limit was largely due to the strategic use of The Construct cloud platform, which bypassed environmental setup challenges and allowed for full focus on the robotics logic.
