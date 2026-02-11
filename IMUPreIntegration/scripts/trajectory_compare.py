#!/usr/bin/env python3
"""
Trajectory comparison tool.
Subscribes to /kiss/trajectory (Path), /pose (Odometry), /odom (Odometry)
and plots them in 2D and 3D when the node shuts down or Ctrl+C is pressed.

Usage:
  rosrun imu_preintegration trajectory_compare.py

Or add to launch file:
  <node pkg="imu_preintegration" type="trajectory_compare.py"
        name="trajectory_compare" output="screen"/>
"""

import rospy
import numpy as np
import matplotlib
matplotlib.use('Agg')  # non-interactive backend for saving
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Odometry, Path
import os
import signal
import sys


class TrajectoryCompare:
    def __init__(self):
        rospy.init_node('trajectory_compare', anonymous=True)

        # Storage: lists of [x, y, z, timestamp]
        self.kiss_pts = []   # from /kiss/trajectory (Path)
        self.pose_pts = []   # from /pose (Odometry) — fused
        self.odom_pts = []   # from /odom (Odometry) — IMU-only

        # Also track orientations for /pose and /odom (quaternion w,x,y,z)
        self.pose_quats = []
        self.odom_quats = []

        # Subscribers
        rospy.Subscriber('/kiss/trajectory', Path, self.kiss_cb)
        rospy.Subscriber('/pose', Odometry, self.pose_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        # Save path
        self.save_dir = os.path.expanduser('~/trajectory_plots')
        os.makedirs(self.save_dir, exist_ok=True)

        # Handle shutdown
        rospy.on_shutdown(self.on_shutdown)
        signal.signal(signal.SIGINT, self._signal_handler)

        rospy.loginfo("TrajectoryCompare ready. Collecting data...")
        rospy.loginfo("Plots will be saved to %s on shutdown", self.save_dir)

    def _signal_handler(self, sig, frame):
        rospy.signal_shutdown("Ctrl+C")
        sys.exit(0)

    def kiss_cb(self, msg):
        """Path msg — extract the latest pose from the path."""
        if len(msg.poses) == 0:
            return
        # Store the last pose in the path (most recent)
        p = msg.poses[-1].pose.position
        t = msg.header.stamp.to_sec()
        self.kiss_pts.append([p.x, p.y, p.z, t])

    def pose_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.header.stamp.to_sec()
        self.pose_pts.append([p.x, p.y, p.z, t])
        self.pose_quats.append([q.w, q.x, q.y, q.z])

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.header.stamp.to_sec()
        self.odom_pts.append([p.x, p.y, p.z, t])
        self.odom_quats.append([q.w, q.x, q.y, q.z])

    def on_shutdown(self):
        rospy.loginfo("Shutdown — generating plots...")
        rospy.loginfo("  kiss: %d pts, pose: %d pts, odom: %d pts",
                      len(self.kiss_pts), len(self.pose_pts), len(self.odom_pts))

        if len(self.kiss_pts) < 2 and len(self.pose_pts) < 2:
            rospy.logwarn("Not enough data to plot")
            return

        kiss = np.array(self.kiss_pts) if self.kiss_pts else np.zeros((0, 4))
        pose = np.array(self.pose_pts) if self.pose_pts else np.zeros((0, 4))
        odom = np.array(self.odom_pts) if self.odom_pts else np.zeros((0, 4))

        # Downsample odom for plotting (it's 200Hz, too dense)
        if len(odom) > 2000:
            step = len(odom) // 2000
            odom_plot = odom[::step]
        else:
            odom_plot = odom

        # ---- Plot 1: XY top-down view ----
        fig1, ax1 = plt.subplots(1, 1, figsize=(12, 10))
        if len(kiss) > 0:
            ax1.plot(kiss[:, 0], kiss[:, 1], 'g-', linewidth=2, label='KISS-ICP', alpha=0.8)
        if len(pose) > 0:
            ax1.plot(pose[:, 0], pose[:, 1], 'r-', linewidth=1.5, label='Fused (/pose)', alpha=0.8)
        if len(odom_plot) > 0:
            ax1.plot(odom_plot[:, 0], odom_plot[:, 1], 'b-', linewidth=0.5, label='IMU-only (/odom)', alpha=0.4)
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectory Comparison — XY View')
        ax1.legend(fontsize=12)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)
        path1 = os.path.join(self.save_dir, 'trajectory_xy.png')
        fig1.savefig(path1, dpi=150, bbox_inches='tight')
        rospy.loginfo("Saved: %s", path1)

        # ---- Plot 2: XZ side view ----
        fig2, ax2 = plt.subplots(1, 1, figsize=(12, 6))
        if len(kiss) > 0:
            ax2.plot(kiss[:, 0], kiss[:, 2], 'g-', linewidth=2, label='KISS-ICP')
        if len(pose) > 0:
            ax2.plot(pose[:, 0], pose[:, 2], 'r-', linewidth=1.5, label='Fused (/pose)')
        if len(odom_plot) > 0:
            ax2.plot(odom_plot[:, 0], odom_plot[:, 2], 'b-', linewidth=0.5, label='IMU-only (/odom)', alpha=0.4)
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Z (m)')
        ax2.set_title('Trajectory Comparison — XZ Side View (check vertical drift)')
        ax2.legend(fontsize=12)
        ax2.grid(True, alpha=0.3)
        path2 = os.path.join(self.save_dir, 'trajectory_xz.png')
        fig2.savefig(path2, dpi=150, bbox_inches='tight')
        rospy.loginfo("Saved: %s", path2)

        # ---- Plot 3: 3D view ----
        fig3 = plt.figure(figsize=(12, 10))
        ax3 = fig3.add_subplot(111, projection='3d')
        if len(kiss) > 0:
            ax3.plot(kiss[:, 0], kiss[:, 1], kiss[:, 2], 'g-', linewidth=2, label='KISS-ICP')
        if len(pose) > 0:
            ax3.plot(pose[:, 0], pose[:, 1], pose[:, 2], 'r-', linewidth=1.5, label='Fused (/pose)')
        if len(odom_plot) > 0:
            ax3.plot(odom_plot[:, 0], odom_plot[:, 1], odom_plot[:, 2], 'b-', linewidth=0.3, label='IMU-only', alpha=0.3)
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Y (m)')
        ax3.set_zlabel('Z (m)')
        ax3.set_title('Trajectory Comparison — 3D')
        ax3.legend(fontsize=12)
        path3 = os.path.join(self.save_dir, 'trajectory_3d.png')
        fig3.savefig(path3, dpi=150, bbox_inches='tight')
        rospy.loginfo("Saved: %s", path3)

        # ---- Plot 4: XYZ vs time ----
        fig4, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
        labels = ['X (m)', 'Y (m)', 'Z (m)']

        for i, (ax, label) in enumerate(zip(axes, labels)):
            if len(kiss) > 0:
                t0 = kiss[0, 3]
                ax.plot(kiss[:, 3] - t0, kiss[:, i], 'g-', linewidth=2, label='KISS-ICP')
            else:
                t0 = pose[0, 3] if len(pose) > 0 else 0

            if len(pose) > 0:
                ax.plot(pose[:, 3] - t0, pose[:, i], 'r-', linewidth=1.5, label='Fused')
            if len(odom_plot) > 0:
                ax.plot(odom_plot[:, 3] - t0, odom_plot[:, i], 'b-', linewidth=0.3, label='IMU-only', alpha=0.4)
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)
            if i == 0:
                ax.legend(fontsize=10)

        axes[-1].set_xlabel('Time (s)')
        fig4.suptitle('Position Components vs Time', fontsize=14)
        path4 = os.path.join(self.save_dir, 'trajectory_vs_time.png')
        fig4.savefig(path4, dpi=150, bbox_inches='tight')
        rospy.loginfo("Saved: %s", path4)

        # ---- Plot 5: Error between KISS-ICP and fused ----
        if len(kiss) > 5 and len(pose) > 5:
            fig5, axes5 = plt.subplots(2, 1, figsize=(14, 8))

            # Interpolate pose to kiss timestamps for error computation
            from scipy.interpolate import interp1d

            kiss_t = kiss[:, 3]
            pose_t = pose[:, 3]

            # Only compute where timestamps overlap
            t_start = max(kiss_t[0], pose_t[0])
            t_end   = min(kiss_t[-1], pose_t[-1])

            if t_end > t_start:
                mask_k = (kiss_t >= t_start) & (kiss_t <= t_end)
                mask_p = (pose_t >= t_start) & (pose_t <= t_end)

                kiss_f = kiss[mask_k]
                pose_f = pose[mask_p]

                # Interpolate fused onto KISS timestamps
                interp_x = interp1d(pose_f[:, 3], pose_f[:, 0], fill_value='extrapolate')
                interp_y = interp1d(pose_f[:, 3], pose_f[:, 1], fill_value='extrapolate')
                interp_z = interp1d(pose_f[:, 3], pose_f[:, 2], fill_value='extrapolate')

                err_x = kiss_f[:, 0] - interp_x(kiss_f[:, 3])
                err_y = kiss_f[:, 1] - interp_y(kiss_f[:, 3])
                err_z = kiss_f[:, 2] - interp_z(kiss_f[:, 3])
                err_3d = np.sqrt(err_x**2 + err_y**2 + err_z**2)

                t_rel = kiss_f[:, 3] - kiss_f[0, 3]

                axes5[0].plot(t_rel, err_x, 'r-', label='X error', alpha=0.7)
                axes5[0].plot(t_rel, err_y, 'g-', label='Y error', alpha=0.7)
                axes5[0].plot(t_rel, err_z, 'b-', label='Z error', alpha=0.7)
                axes5[0].set_ylabel('Error (m)')
                axes5[0].set_title('Position Error: KISS-ICP minus Fused')
                axes5[0].legend()
                axes5[0].grid(True, alpha=0.3)

                axes5[1].plot(t_rel, err_3d, 'k-', linewidth=1.5)
                axes5[1].set_ylabel('3D Error (m)')
                axes5[1].set_xlabel('Time (s)')
                axes5[1].set_title('Euclidean Distance Error (mean=%.3f m)' % np.mean(err_3d))
                axes5[1].grid(True, alpha=0.3)

                path5 = os.path.join(self.save_dir, 'trajectory_error.png')
                fig5.savefig(path5, dpi=150, bbox_inches='tight')
                rospy.loginfo("Saved: %s", path5)

        # ---- Save raw data as CSV ----
        if len(kiss) > 0:
            np.savetxt(os.path.join(self.save_dir, 'kiss_trajectory.csv'),
                       kiss, delimiter=',', header='x,y,z,timestamp', comments='')
        if len(pose) > 0:
            np.savetxt(os.path.join(self.save_dir, 'fused_trajectory.csv'),
                       pose, delimiter=',', header='x,y,z,timestamp', comments='')
        if len(odom) > 0:
            np.savetxt(os.path.join(self.save_dir, 'imu_trajectory.csv'),
                       odom, delimiter=',', header='x,y,z,timestamp', comments='')
        rospy.loginfo("CSV data saved to %s", self.save_dir)

        plt.close('all')
        rospy.loginfo("Done! Check %s for plots.", self.save_dir)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = TrajectoryCompare()
    node.run()
