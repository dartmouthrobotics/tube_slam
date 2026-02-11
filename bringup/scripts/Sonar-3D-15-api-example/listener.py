#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

def callback(msg):
    points_count = msg.width * msg.height
    rospy.loginfo(f"Received PointCloud2 with {points_count} points")

def listener():
    rospy.init_node('pointcloud_listener', anonymous=True)
    
    # Replace '/your_topic' with the exact topic name you confirmed
    rospy.Subscriber('/sonar3d/point_cloud', PointCloud2, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
