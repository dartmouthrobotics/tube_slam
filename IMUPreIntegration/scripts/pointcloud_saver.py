#!/usr/bin/env python3
"""
Save KISS-ICP point clouds to PCD/PLY files.

Subscribes to /kiss/frame (per-frame registered cloud) and accumulates
into a global point cloud. Also saves /kiss/local_map snapshots.
Writes files on shutdown (Ctrl+C or roslaunch exit).
"""
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import os


class PointCloudSaver:
    def __init__(self):
        rospy.init_node('pointcloud_saver')

        self.save_dir = rospy.get_param('~save_dir', '/home/xiaoming/2026_slam/pointcloud')
        self.downsample_voxel = rospy.get_param('~downsample_voxel', 0.05)
        self.max_points = rospy.get_param('~max_points', 5000000)

        os.makedirs(self.save_dir, exist_ok=True)

        # Accumulated global cloud from /kiss/frame
        self.global_points = []
        self.global_count = 0
        self.frame_count = 0

        # Latest local map
        self.latest_local_map = None

        # Subscribers
        self.frame_sub = rospy.Subscriber('/kiss/frame', PointCloud2, self.frame_callback)
        self.local_map_sub = rospy.Subscriber('/kiss/local_map', PointCloud2, self.local_map_callback)

        # Save on shutdown
        rospy.on_shutdown(self.save_all)

        rospy.loginfo("PointCloudSaver ready. Saving to: %s", self.save_dir)
        rospy.loginfo("  Voxel downsample: %.3f m (0=off)", self.downsample_voxel)
        rospy.loginfo("  Max points: %d", self.max_points)

    def pointcloud2_to_numpy(self, msg):
        """Convert PointCloud2 message to Nx3 numpy array."""
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        if len(points) == 0:
            return np.empty((0, 3))
        return np.array(points, dtype=np.float32)

    def frame_callback(self, msg):
        """Accumulate each registered frame into global cloud."""
        pts = self.pointcloud2_to_numpy(msg)
        if pts.shape[0] == 0:
            return

        self.global_points.append(pts)
        self.global_count += pts.shape[0]
        self.frame_count += 1

        if self.frame_count % 50 == 0:
            rospy.loginfo("PointCloudSaver: %d frames, %d points accumulated",
                          self.frame_count, self.global_count)

        # If too many points, trigger intermediate downsampling
        if self.global_count > self.max_points * 1.5:
            rospy.loginfo("Intermediate downsample at %d points...", self.global_count)
            self._consolidate_and_downsample()

    def local_map_callback(self, msg):
        """Store latest local map."""
        self.latest_local_map = msg

    def _consolidate_and_downsample(self):
        """Merge all accumulated arrays and optionally voxel downsample."""
        if len(self.global_points) == 0:
            return

        all_pts = np.vstack(self.global_points)

        if self.downsample_voxel > 0:
            all_pts = self.voxel_downsample(all_pts, self.downsample_voxel)

        self.global_points = [all_pts]
        self.global_count = all_pts.shape[0]
        rospy.loginfo("After downsample: %d points", self.global_count)

    def voxel_downsample(self, points, voxel_size):
        """Simple voxel grid downsampling using numpy."""
        if points.shape[0] == 0:
            return points

        # Quantize to voxel grid
        voxel_indices = np.floor(points / voxel_size).astype(np.int32)

        # Use unique voxels - take one point per voxel
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        downsampled = points[unique_idx]

        rospy.loginfo("Voxel downsample: %d -> %d points (voxel=%.3fm)",
                      points.shape[0], downsampled.shape[0], voxel_size)
        return downsampled

    def save_ply(self, filepath, points):
        """Save Nx3 points as binary PLY file."""
        n = points.shape[0]
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            "element vertex {}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "end_header\n"
        ).format(n)
        with open(filepath, 'wb') as f:
            f.write(header.encode('ascii'))
            f.write(points.astype(np.float32).tobytes())

        rospy.loginfo("Saved PLY: %s (%d points, %.1f MB)",
                      filepath, n, os.path.getsize(filepath) / 1e6)

    def save_pcd(self, filepath, points):
        """Save Nx3 points as ASCII PCD file."""
        n = points.shape[0]
        header = (
            "# .PCD v0.7 - Point Cloud Data\n"
            "VERSION 0.7\n"
            "FIELDS x y z\n"
            "SIZE 4 4 4\n"
            "TYPE F F F\n"
            "COUNT 1 1 1\n"
            "WIDTH {}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS {}\n"
            "DATA ascii\n"
        ).format(n, n)
        with open(filepath, 'w') as f:
            f.write(header)
            for p in points:
                f.write("{:.6f} {:.6f} {:.6f}\n".format(p[0], p[1], p[2]))

        rospy.loginfo("Saved PCD: %s (%d points, %.1f MB)",
                      filepath, n, os.path.getsize(filepath) / 1e6)

    def save_all(self):
        """Called on shutdown - save all accumulated clouds."""
        rospy.loginfo("=" * 50)
        rospy.loginfo("PointCloudSaver: Saving point clouds...")

        # Save global accumulated cloud
        if len(self.global_points) > 0:
            self._consolidate_and_downsample()
            all_pts = self.global_points[0]

            # Save as PLY (compact binary)
            ply_path = os.path.join(self.save_dir, "global_cloud.ply")
            self.save_ply(ply_path, all_pts)

            # Save as PCD (for PCL tools)
            pcd_path = os.path.join(self.save_dir, "global_cloud.pcd")
            self.save_pcd(pcd_path, all_pts)

            # Save as numpy for analysis
            npy_path = os.path.join(self.save_dir, "global_cloud.npy")
            np.save(npy_path, all_pts)
            rospy.loginfo("Saved NPY: %s", npy_path)
        else:
            rospy.logwarn("No frame data accumulated!")

        # Save latest local map
        if self.latest_local_map is not None:
            local_pts = self.pointcloud2_to_numpy(self.latest_local_map)
            if local_pts.shape[0] > 0:
                ply_path = os.path.join(self.save_dir, "local_map.ply")
                self.save_ply(ply_path, local_pts)
                pcd_path = os.path.join(self.save_dir, "local_map.pcd")
                self.save_pcd(pcd_path, local_pts)
        else:
            rospy.logwarn("No local map received!")

        rospy.loginfo("PointCloudSaver: Done. Files in %s", self.save_dir)
        rospy.loginfo("=" * 50)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = PointCloudSaver()
    node.run()
