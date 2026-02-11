#!/usr/bin/env python3
"""
Post-processing wrapper: waits for pointcloud_saver to finish,
then runs visualize_pointcloud.py to generate PNG images.
Runs as a ROS node so it can detect shutdown.
"""
import rospy
import subprocess
import os
import sys
import time


def main():
    rospy.init_node('pointcloud_visualizer', disable_signals=True)

    pc_dir = rospy.get_param('~pointcloud_dir', '/home/xiaoming/2026_slam/pointcloud')
    output_dir = rospy.get_param('~output_dir', pc_dir)
    voxel = rospy.get_param('~voxel', 0.1)
    color = rospy.get_param('~color', 'height')
    dpi = rospy.get_param('~dpi', 150)

    # Find visualize_pointcloud.py relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    viz_script = os.path.join(script_dir, 'visualize_pointcloud.py')
    if not os.path.exists(viz_script):
        viz_script = '/home/xiaoming/2026_slam/catkin_ws/src/IMUPreIntegration/scripts/visualize_pointcloud.py'

    if not os.path.exists(viz_script):
        rospy.logerr("visualize_pointcloud.py not found at %s", viz_script)
        return

    rospy.loginfo("Post-processor ready. Will generate PNGs on shutdown.")
    rospy.loginfo("  Pointcloud dir: %s", pc_dir)
    rospy.loginfo("  Output dir: %s", output_dir)

    # Wait for shutdown
    rospy.spin()

    # --- Post-shutdown processing ---
    rospy.loginfo("=" * 50)
    rospy.loginfo("Shutdown detected. Waiting for files...")

    # Wait for pointcloud_saver to finish writing (up to 30s)
    cloud_files = ['global_cloud.ply', 'global_cloud.pcd', 'local_map.ply']
    target = os.path.join(pc_dir, 'global_cloud.ply')

    for i in range(60):  # 30 seconds max
        if os.path.exists(target) and os.path.getsize(target) > 0:
            time.sleep(2)  # extra wait for file to finish writing
            break
        time.sleep(0.5)
    else:
        rospy.logwarn("Timed out waiting for %s", target)
        return

    # Build file list
    files_to_viz = []
    for f in cloud_files:
        path = os.path.join(pc_dir, f)
        if os.path.exists(path) and os.path.getsize(path) > 100:
            files_to_viz.append(path)

    if not files_to_viz:
        rospy.logwarn("No point cloud files found in %s", pc_dir)
        return

    rospy.loginfo("Generating visualizations for %d files...", len(files_to_viz))

    # Run visualizer for each file
    for filepath in files_to_viz:
        rospy.loginfo("Visualizing: %s", filepath)
        cmd = [
            sys.executable, viz_script,
            filepath,
            '--voxel', str(voxel),
            '--color', color,
            '--dpi', str(dpi),
            '-o', output_dir,
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            if result.returncode == 0:
                rospy.loginfo("  Done: %s", os.path.basename(filepath))
                if result.stdout:
                    for line in result.stdout.strip().split('\n'):
                        rospy.loginfo("  %s", line)
            else:
                rospy.logerr("  Failed: %s", result.stderr[-200:] if result.stderr else "unknown")
        except subprocess.TimeoutExpired:
            rospy.logerr("  Timed out processing %s", filepath)

    # Overlay comparison if both global and local exist
    global_path = os.path.join(pc_dir, 'global_cloud.ply')
    local_path = os.path.join(pc_dir, 'local_map.ply')
    if os.path.exists(global_path) and os.path.exists(local_path):
        rospy.loginfo("Generating overlay comparison...")
        cmd = [
            sys.executable, viz_script,
            global_path, local_path,
            '--overlay',
            '--voxel', str(voxel),
            '-o', output_dir,
        ]
        try:
            subprocess.run(cmd, capture_output=True, text=True, timeout=120)
            rospy.loginfo("  Overlay done.")
        except Exception as e:
            rospy.logerr("  Overlay failed: %s", e)

    rospy.loginfo("=" * 50)
    rospy.loginfo("All visualizations saved to: %s", output_dir)
    rospy.loginfo("=" * 50)


if __name__ == '__main__':
    main()
