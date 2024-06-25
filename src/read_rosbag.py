#!/usr/bin/python3
import rosbag
import rospy
from std_msgs.msg import String

# Path to the .bag file
bag_file_path = '/home/rishabh/Robot/Octo_robot/ros_ws/src/openvla_real_expr/trajectory_with_images.bag'

# Open the bag file
bag = rosbag.Bag(bag_file_path)

print(bag_file_path)

# Iterate through the messages in the bag
for topic, msg, t in bag.read_messages(topics=['/joint_states']):
    print(f"Topic: {topic}")
    print(f"Message: {msg}")
    print(f"Timestamp: {t}")

# Close the bag file
bag.close()

