#!/usr/bin/env python3
import rospy
import gtsam
import numpy as np
from nav_msgs.msg import Odometry
import tf2_ros

class TransformTester:
    def __init__(self):
        rospy.init_node('transform_tester')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber('/kiss/odometry', Odometry, self.test_callback)
        self.count = 0
        
    def odom_to_pose3(self, odom_msg):
        t = odom_msg.pose.pose.position
        translation = [t.x, t.y, t.z]
        q = odom_msg.pose.pose.orientation
        rotation = gtsam.Rot3.Quaternion(q.w, q.x, q.y, q.z)
        return gtsam.Pose3(rotation, translation)
    
    def test_callback(self, msg):
        self.count += 1
        if self.count % 10 != 0:  # Print every 10th message
            return
            
        # Original KISS-ICP pose
        pose = self.odom_to_pose3(msg)
        orig_pos = pose.translation()
        
        # Get TF transform
        try:
            transform = self.tf_buffer.lookup_transform('MiniAHRS', 'sonar_3d', 
                                                       rospy.Time(0), rospy.Duration(1.0))
            q = transform.transform.rotation
            t = transform.transform.translation
            R_gtsam = gtsam.Rot3.Quaternion(q.w, q.x, q.y, q.z)
            t_gtsam = [t.x, t.y, t.z]
            T_sonar2imu = gtsam.Pose3(R_gtsam, t_gtsam)
            
            # Test BOTH transform directions
            # Method 1: T_imu = T_s2i^-1 * T_sonar * T_s2i
            trans1 = T_sonar2imu.inverse() * pose * T_sonar2imu
            pos1 = trans1.translation()
            
            # Method 2: T_imu = T_s2i * T_sonar
            trans2 = T_sonar2imu * pose
            pos2 = trans2.translation()
            
            # Method 3: T_imu = T_sonar * T_s2i
            trans3 = pose * T_sonar2imu
            pos3 = trans3.translation()
            
            # Method 4: T_imu = T_s2i^-1 * T_sonar  
            trans4 = T_sonar2imu.inverse() * pose
            pos4 = trans4.translation()
            
            print(f"\n=== Frame {self.count} ===")
            print(f"Original (sonar): [{orig_pos[0]:.3f}, {orig_pos[1]:.3f}, {orig_pos[2]:.3f}]")
            print(f"Method 1 (T^-1*P*T): [{pos1[0]:.3f}, {pos1[1]:.3f}, {pos1[2]:.3f}] - mag ratio: {np.linalg.norm(pos1)/np.linalg.norm(orig_pos):.2f}x")
            print(f"Method 2 (T*P):      [{pos2[0]:.3f}, {pos2[1]:.3f}, {pos2[2]:.3f}] - mag ratio: {np.linalg.norm(pos2)/np.linalg.norm(orig_pos):.2f}x")
            print(f"Method 3 (P*T):      [{pos3[0]:.3f}, {pos3[1]:.3f}, {pos3[2]:.3f}] - mag ratio: {np.linalg.norm(pos3)/np.linalg.norm(orig_pos):.2f}x")
            print(f"Method 4 (T^-1*P):   [{pos4[0]:.3f}, {pos4[1]:.3f}, {pos4[2]:.3f}] - mag ratio: {np.linalg.norm(pos4)/np.linalg.norm(orig_pos):.2f}x")
            
        except Exception as e:
            rospy.logwarn(f"Transform failed: {e}")

if __name__ == '__main__':
    tester = TransformTester()
    rospy.spin()