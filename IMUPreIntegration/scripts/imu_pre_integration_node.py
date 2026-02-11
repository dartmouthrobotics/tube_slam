#!/usr/bin/env python3
"""
IMU + KISS-ICP Sonar Odometry Fusion using GTSAM iSAM2.

Strategy: rotate IMU measurements into sonar frame BEFORE preintegration,
so all GTSAM states live in the same frame as KISS-ICP. No adjoint
transformation of poses needed.
"""
import rospy
import gtsam
import numpy as np
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Vector3, TransformStamped
from gtsam import (
    PreintegratedImuMeasurements,
    ImuFactor,
    BetweenFactorPose3,
    PriorFactorConstantBias,
)
from gtsam.symbol_shorthand import B, V, X
import tf2_ros
from tf.transformations import quaternion_from_matrix
import math


def rotation_matrix_align(a, b):
    """Return 3x3 rotation R such that R @ a = b (both unit vectors)."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = np.dot(a, b)
    if np.linalg.norm(v) < 1e-8:
        return np.eye(3) if c > 0 else (
            2.0 * np.outer((lambda u: u / np.linalg.norm(u))(
                np.cross(a, np.array([1, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1, 0]))
            ), (lambda u: u / np.linalg.norm(u))(
                np.cross(a, np.array([1, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1, 0]))
            )) - np.eye(3)
        )
    skew = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + skew + skew @ skew / (1.0 + c)


class ImuOdometrySubscriber:
    def __init__(self):
        rospy.init_node('imu_odometry_subscriber')

        # =============================================================
        # Extrinsic calibration: IMU <-> Sonar
        #
        # result euler angle(RPY) : 3.12156 0.0747417 1.56107
        # result extrinsic rotation matrix :
        #  0.00969904    0.999767    0.019301
        #    0.997161 -0.00822878  -0.0748484
        #  -0.0746721   0.0199722   -0.997008
        #
        # Convention: R_imu_to_sonar  (v_sonar = R * v_imu)
        # =============================================================
        self.R_imu_to_sonar = np.array([
            [ 0.00969904,  0.999767,    0.019301],
            [ 0.997161,   -0.00822878, -0.0748484],
            [-0.0746721,   0.0199722,  -0.997008],
        ]).T

        # self.R_imu_to_sonar = np.eye(3)

        rospy.loginfo("Extrinsic R_imu_to_sonar loaded (det=%.4f)",
                       np.linalg.det(self.R_imu_to_sonar))

        # =============================================================
        # IMU noise parameters — from imu.yaml
        # =============================================================
        scale_imu_noise = 100.0

        acc_noise_density  = 0.0006479162116471713
        acc_random_walk    = 2.3703449414053308e-05
        gyro_noise_density = 0.006028235905489944
        gyro_random_walk   = 7.395639173156115e-05

        self.acc_noise    = acc_noise_density  * scale_imu_noise
        self.gyro_noise   = gyro_noise_density * scale_imu_noise
        self.acc_bias_rw  = acc_random_walk    * scale_imu_noise
        self.gyro_bias_rw = gyro_random_walk   * scale_imu_noise

        self.noiseModelBetweenBias = np.array(
            [self.acc_bias_rw] * 3 + [self.gyro_bias_rw] * 3
        )

        # Mini AHRS outputs gyro in deg/s
        self.gyro_deg_to_rad = True

        # =============================================================
        # ROS
        # =============================================================
        self.sonar_frame_id = "sonar_3d"
        self.imu_frame_id   = "MiniAHRS"

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.imu_subscriber  = rospy.Subscriber('/mini_ahrs_ros/imu', Imu, self.imu_callback)
        self.odom_subscriber = rospy.Subscriber('/kiss/odometry', Odometry, self.odom_callback)
        self.pose_publisher  = rospy.Publisher('/pose', Odometry, queue_size=10)
        self.odom_publisher  = rospy.Publisher('/odom', Odometry, queue_size=10)

        # =============================================================
        # Gravity calibration
        # =============================================================
        self.calibrating         = True
        self.calib_accel_samples = []
        self.calib_gyro_samples  = []
        self.num_calib_samples   = 200
        self.gravity_magnitude   = 9.81
        self.imu_params          = None
        self.R0 = gtsam.Rot3(np.eye(3))

        # =============================================================
        # GTSAM state — all in sonar/odom frame
        # =============================================================
        self.prev_bias     = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))
        self.prev_imu_time = None
        self.prev_time     = None
        self.prev_pose     = None
        self.prev_velocity = np.zeros(3)
        self.preintegrated = None

        self.prev_odom_pose = None

        self.graph            = gtsam.NonlinearFactorGraph()
        self.initial_estimate = gtsam.Values()
        self.imu_cnt          = 0
        self.fixed_dt         = 1.0 / 200.0

        self.consecutive_failures     = 0
        self.max_consecutive_failures = 3
        self.isam = gtsam.ISAM2()

        rospy.loginfo("Waiting for gravity calibration (%d samples)...",
                       self.num_calib_samples)

    # ----------------------------------------------------------
    # IMU measurement rotation
    # ----------------------------------------------------------
    def rotate_imu_to_sonar(self, accel_imu, gyro_imu):
        """Rotate raw IMU readings from IMU body frame into sonar frame."""
        return self.R_imu_to_sonar @ accel_imu, self.R_imu_to_sonar @ gyro_imu

    def convert_gyro(self, gyro):
        if self.gyro_deg_to_rad:
            return gyro * (math.pi / 180.0)
        return gyro

    # ----------------------------------------------------------
    # Gravity calibration (in sonar frame)
    # ----------------------------------------------------------
    def calibrate_gravity(self):
        accel_arr = np.array(self.calib_accel_samples)
        gyro_arr  = np.array(self.calib_gyro_samples)

        mean_accel   = np.mean(accel_arr, axis=0)
        mean_gyro    = np.mean(gyro_arr,  axis=0)
        gravity_norm = np.linalg.norm(mean_accel)

        rospy.loginfo("Gravity in sonar frame (already rotated): %s", mean_accel)
        rospy.loginfo("Expected: dominant component should be along sonar Z-axis")

        rospy.loginfo("=" * 50)
        rospy.loginfo("GRAVITY CALIBRATION (sonar frame)")
        rospy.loginfo("  mean_accel = %s  |a| = %.4f", mean_accel, gravity_norm)
        rospy.loginfo("  mean_gyro  = %s", mean_gyro)

        self.gravity_magnitude = gravity_norm

        # R0: aligns GTSAM's [0,0,-g] with measured gravity in sonar body
        g_nav  = np.array([0.0, 0.0, -1.0])
        g_body = -mean_accel / gravity_norm
        R0_mat = rotation_matrix_align(g_nav, g_body)
        self.R0 = gtsam.Rot3(R0_mat)
        rospy.loginfo("  R0:\n%s", R0_mat)

        g_in_sonar_frame = -mean_accel  # gravity direction in sonar body frame at rest

        g_nav = -mean_accel  # mean_accel is already in sonar frame
        rospy.loginfo("  Using custom gravity vector in nav frame: %s", g_nav)
        self.imu_params = gtsam.PreintegrationParams(g_nav)

        self.imu_params.setAccelerometerCovariance(np.eye(3) * self.acc_noise ** 2)
        self.imu_params.setGyroscopeCovariance(np.eye(3) * self.gyro_noise ** 2)
        self.imu_params.setIntegrationCovariance(np.eye(3) * 1e-4)

        # Initial bias
        a_expected      = R0_mat.T @ np.array([0.0, 0.0, self.gravity_magnitude])
        accel_bias_init = np.zeros(3)
        gyro_bias_init  = mean_gyro
        rospy.loginfo("  accel_bias_init = %s", accel_bias_init)
        rospy.loginfo("  gyro_bias_init  = %s", gyro_bias_init)

        rospy.loginfo("  Gravity in sonar frame: %s", mean_accel)
        rospy.loginfo("  Gravity should be ~[0, 0, ±9.81] if sonar Z is vertical")
        rospy.loginfo("  Actual dominant axis: %s (value=%.2f)",
                        ['X','Y','Z'][np.argmax(np.abs(mean_accel))],
                        mean_accel[np.argmax(np.abs(mean_accel))])

        self.prev_bias = gtsam.imuBias.ConstantBias(accel_bias_init, gyro_bias_init)
        self.calibrating = False
        rospy.loginfo("Calibration done. Fusion active.")
        rospy.loginfo("=" * 50)

    # ----------------------------------------------------------
    # IMU callback
    # ----------------------------------------------------------
    def imu_callback(self, msg):
        accel_imu = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        gyro_imu = self.convert_gyro(np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]))

        # Rotate into sonar frame
        accel, gyro = self.rotate_imu_to_sonar(accel_imu, gyro_imu)

        # --- Calibration phase ---
        if self.calibrating:
            self.calib_accel_samples.append(accel.copy())
            self.calib_gyro_samples.append(gyro.copy())
            if len(self.calib_accel_samples) >= self.num_calib_samples:
                self.calibrate_gravity()
            self.prev_imu_time = msg.header.stamp.to_sec()
            return

        # --- Normal phase ---
        if self.prev_imu_time is None or self.prev_time is None:
            self.prev_imu_time = msg.header.stamp.to_sec()
            return

        dt = msg.header.stamp.to_sec() - self.prev_imu_time
        if dt <= 0:
            dt = self.fixed_dt

        if self.preintegrated is None:
            self.preintegrated = PreintegratedImuMeasurements(
                self.imu_params, self.prev_bias
            )

        self.preintegrated.integrateMeasurement(accel, gyro, dt)
        self.prev_imu_time = msg.header.stamp.to_sec()

        # High-rate IMU-only prediction
        if self.prev_pose is not None:
            try:
                state = self.preintegrated.predict(
                    gtsam.NavState(self.prev_pose, self.prev_velocity),
                    self.prev_bias,
                )
                self.odom_publisher.publish(self.pose3_to_odom(
                    state.pose(), state.velocity(),
                    stamp=msg.header.stamp,
                    frame_id="odom", child_frame_id="fused_frame",
                ))
            except Exception as e:
                rospy.logwarn_throttle(5.0, "IMU predict: %s", e)

    # ----------------------------------------------------------
    # KISS-ICP odometry callback
    # ----------------------------------------------------------
    def odom_callback(self, msg):
        if self.calibrating:
            return

        odom_pose = self.odom_to_pose3(msg)  # in sonar/odom frame
        curr_time = msg.header.stamp.to_sec()

        # --- First frame ---
        if self.prev_pose is None:
            self.prev_time      = curr_time
            self.prev_odom_pose = odom_pose
            self.prev_pose      = odom_pose
            self.prev_velocity  = np.zeros(3)
            self.add_priors(self.prev_pose, self.prev_velocity)
            rospy.loginfo("First pose initialized")
            return

        if self.preintegrated is None:
            rospy.logwarn_throttle(5.0, "No IMU data yet")
            return

        deltaTij = self.preintegrated.deltaTij()
        if deltaTij <= 0:
            return
        if deltaTij > 3.0:
            rospy.logwarn("deltaTij=%.3f too large, reset", deltaTij)
            self.preintegrated.resetIntegrationAndSetBias(self.prev_bias)
            self.prev_odom_pose = odom_pose
            return

        self.prev_time = curr_time
        self.imu_cnt += 1

        # Relative pose from KISS-ICP; sonar frame, no transform needed
        relative_pose = self.prev_odom_pose.inverse().compose(odom_pose)
        self.prev_odom_pose = odom_pose

        sonar_velocity = relative_pose.translation() / deltaTij

        # IMU prediction for initial estimate
        prev_state = gtsam.NavState(self.prev_pose, self.prev_velocity)
        try:
            predicted = self.preintegrated.predict(prev_state, self.prev_bias)
        except Exception as e:
            rospy.logerr("IMU predict fail: %s", e)
            self.preintegrated.resetIntegrationAndSetBias(self.prev_bias)
            self.imu_cnt -= 1
            return

        # Diagnostics
        if self.imu_cnt % 10 == 0:
            rospy.loginfo("Frame %d  dT=%.4f  IMU_v=%.3f  sonar_v=%.3f",
                          self.imu_cnt, deltaTij,
                          np.linalg.norm(predicted.velocity()),
                          np.linalg.norm(sonar_velocity))

        # Check bias divergence
        ba = np.linalg.norm(self.prev_bias.accelerometer())
        bg = np.linalg.norm(self.prev_bias.gyroscope())
        if ba > 3.0 or bg > 1.0:
            rospy.logwarn("Bias diverged (ba=%.2f bg=%.2f), reinit", ba, bg)
            self.reinitialize_isam(odom_pose, curr_time)
            return

        # ---- Build factor graph ----

        # IMU factor
        self.graph.push_back(ImuFactor(
            X(self.imu_cnt - 1), V(self.imu_cnt - 1),
            X(self.imu_cnt),     V(self.imu_cnt),
            B(self.imu_cnt - 1), self.preintegrated,
        ))

        # Bias random walk
        bias_sigma = math.sqrt(deltaTij) * self.noiseModelBetweenBias
        bias_sigma = np.maximum(bias_sigma, 1e-3)
        self.graph.push_back(gtsam.BetweenFactorConstantBias(
            B(self.imu_cnt - 1), B(self.imu_cnt),
            gtsam.imuBias.ConstantBias(),
            gtsam.noiseModel.Diagonal.Sigmas(bias_sigma),
        ))

        # Sonar relative pose factor 
        self.graph.push_back(BetweenFactorPose3(
            X(self.imu_cnt - 1), X(self.imu_cnt),
            relative_pose,
            gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.01, 0.01, 0.01,   # rotation (rad)
                          0.02, 0.02, 0.02])   # translation (m)
    ),
        ))

        # Absolute pose anchor to KISS-ICP 
        self.graph.push_back(gtsam.PriorFactorPose3(
            X(self.imu_cnt), odom_pose,
            gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.05, 0.05, 0.05,    # rotation (rad)
                        0.10, 0.10, 0.10])    # translation (m)
            ),
        ))

        # Soft bias anchor
        self.graph.push_back(PriorFactorConstantBias(
            B(self.imu_cnt), self.prev_bias,
            gtsam.noiseModel.Isotropic.Sigma(6, 0.1),
        ))

        # Velocity prior from sonar
        self.graph.push_back(gtsam.PriorFactorVector(
            V(self.imu_cnt), sonar_velocity,
            gtsam.noiseModel.Isotropic.Sigma(3, 0.2),
        ))

        # Initial estimates
        self.initial_estimate.insert(X(self.imu_cnt), self.prev_pose.compose(relative_pose))
        self.initial_estimate.insert(V(self.imu_cnt), sonar_velocity)
        self.initial_estimate.insert(B(self.imu_cnt), self.prev_bias)

        # ---- Optimize ----
        try:
            self.isam.update(self.graph, self.initial_estimate)
            self.graph.resize(0)
            self.initial_estimate.clear()

            result = self.isam.calculateEstimate()
            self.prev_pose     = result.atPose3(X(self.imu_cnt))
            self.prev_velocity = result.atVector(V(self.imu_cnt))
            self.prev_bias     = result.atConstantBias(B(self.imu_cnt))
            self.consecutive_failures = 0

            self.pose_publisher.publish(self.pose3_to_odom(
                self.prev_pose, self.prev_velocity,
                stamp=msg.header.stamp,
                frame_id="odom", child_frame_id="fused_frame",
            ))
            self.broadcast_tf(self.prev_pose, msg.header.stamp, "odom", "fused_frame")
            self.preintegrated.resetIntegrationAndSetBias(self.prev_bias)

        except (RuntimeError, IndexError) as e:
            rospy.logerr("Optimize fail frame %d: %s", self.imu_cnt, e)
            self.graph.resize(0)
            self.initial_estimate.clear()
            self.consecutive_failures += 1
            self.preintegrated.resetIntegrationAndSetBias(self.prev_bias)
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.reinitialize_isam(odom_pose, curr_time)

    # ----------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------
    def broadcast_tf(self, pose3, stamp, parent, child):
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = parent
        t.child_frame_id  = child
        tr = pose3.translation()
        t.transform.translation.x = tr[0]
        t.transform.translation.y = tr[1]
        t.transform.translation.z = tr[2]
        R = pose3.rotation().matrix()
        q = quaternion_from_matrix([
            [R[0,0], R[0,1], R[0,2], 0],
            [R[1,0], R[1,1], R[1,2], 0],
            [R[2,0], R[2,1], R[2,2], 0],
            [0,      0,      0,      1],
        ])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def reinitialize_isam(self, odom_pose, curr_time):
        self.isam = gtsam.ISAM2()
        self.graph.resize(0)
        self.initial_estimate.clear()
        self.prev_pose      = odom_pose
        self.prev_velocity  = np.zeros(3)
        self.prev_bias      = gtsam.imuBias.ConstantBias(np.zeros(3), np.zeros(3))
        self.prev_odom_pose = odom_pose
        self.imu_cnt += 100
        self.consecutive_failures = 0
        idx = self.imu_cnt
        pn = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        vn = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        bn = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        self.graph.push_back(gtsam.PriorFactorPose3(X(idx), self.prev_pose, pn))
        self.graph.push_back(gtsam.PriorFactorVector(V(idx), self.prev_velocity, vn))
        self.graph.push_back(PriorFactorConstantBias(B(idx), self.prev_bias, bn))
        self.initial_estimate.insert(X(idx), self.prev_pose)
        self.initial_estimate.insert(V(idx), self.prev_velocity)
        self.initial_estimate.insert(B(idx), self.prev_bias)
        try:
            self.isam.update(self.graph, self.initial_estimate)
            self.graph.resize(0)
            self.initial_estimate.clear()
            rospy.loginfo("ISAM2 reinit at %d", idx)
        except (RuntimeError, IndexError) as e:
            rospy.logerr("Reinit fail: %s", e)
            self.graph.resize(0)
            self.initial_estimate.clear()
        if self.preintegrated is not None:
            self.preintegrated.resetIntegrationAndSetBias(self.prev_bias)

    def add_priors(self, initial_pose, initial_velocity):
        pn = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        vn = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)
        bn = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
        self.graph.push_back(gtsam.PriorFactorPose3(X(0), initial_pose, pn))
        self.graph.push_back(gtsam.PriorFactorVector(V(0), initial_velocity, vn))
        self.graph.push_back(PriorFactorConstantBias(B(0), self.prev_bias, bn))
        self.initial_estimate.insert(X(0), initial_pose)
        self.initial_estimate.insert(V(0), initial_velocity)
        self.initial_estimate.insert(B(0), self.prev_bias)
        try:
            self.isam.update(self.graph, self.initial_estimate)
            self.graph.resize(0)
            self.initial_estimate.clear()
            rospy.loginfo("Priors added OK")
        except (RuntimeError, IndexError) as e:
            rospy.logerr("Prior init fail: %s", e)
            self.graph.resize(0)
            self.initial_estimate.clear()

    def pose3_to_odom(self, pose3, velocity=None, frame_id="odom",
                      child_frame_id="base_link", stamp=None):
        odom_msg = Odometry()
        odom_msg.header.frame_id = frame_id
        odom_msg.child_frame_id  = child_frame_id
        odom_msg.header.stamp    = stamp if stamp else rospy.Time.now()
        t = pose3.translation()
        odom_msg.pose.pose.position = Point(*t)
        R = pose3.rotation().matrix()
        q = quaternion_from_matrix([
            [R[0,0], R[0,1], R[0,2], 0],
            [R[1,0], R[1,1], R[1,2], 0],
            [R[2,0], R[2,1], R[2,2], 0],
            [0,      0,      0,      1],
        ])
        odom_msg.pose.pose.orientation = Quaternion(*q)
        if velocity is not None:
            odom_msg.twist.twist.linear = Vector3(*velocity)
        return odom_msg

    def odom_to_pose3(self, odom_msg):
        t = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        return gtsam.Pose3(
            gtsam.Rot3.Quaternion(q.w, q.x, q.y, q.z),
            [t.x, t.y, t.z],
        )

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = ImuOdometrySubscriber()
    node.run()