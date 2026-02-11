import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

bag = rosbag.Bag('../../bags/sonar_data/sonar-recording-2025-07-31-155611.bag')
for topic, msg, t in bag.read_messages(topics=['sonar3d/point_cloud']):
    print(f"Topic: {topic}, Time: {t.to_sec()}")
    # Assuming the message is of type PointCloud2
    # try:
    #     for p in pc2.read_points(msg, field_names=["x", "y", "z", "yaw", "pitch", "distance"], skip_nans=True):
    #         print(p)
    #     break  # remove this if you want all messages
    # except Exception as e:
    #     print(f"Failed to parse: {e}")