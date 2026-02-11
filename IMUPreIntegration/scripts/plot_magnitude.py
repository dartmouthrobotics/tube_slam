import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# INPUT_FILE = 'tank_392_imu_data_1000000.csv'
# INPUT_FILE = 'tank_392_imu_data_1000001_2000000.csv'
INPUT_FILE = 'tank_392_imu_data_2000001_3000000.csv'
# INPUT_FILE = 'tank_392_imu_data_3000001_4000000.csv' 
# INPUT_FILE = 'tank_392_imu_data_4000001_5000000.csv'
# INPUT_FILE = 'tank_483_2025-08-21-19-46-37_0_500_600.csv'

# https://ustc-flicar.github.io/calibration/#imu
# INPUT_FILE = 'Xsens_MTi_G_710_noise.mat'

root, extension = os.path.splitext(INPUT_FILE)

if "csv" in extension:

    df = pd.read_csv(INPUT_FILE)

    time_column = df['%time']

    accel_x = df['field.linear_acceleration.x']
    accel_y = df['field.linear_acceleration.y']
    accel_z = df['field.linear_acceleration.z']

    accel_x = accel_x[accel_x != 0]
    accel_y = accel_y[accel_y != 0]
    accel_z = accel_z[accel_z != 0]

    angular_x = df['field.angular_velocity.x']
    angular_y = df['field.angular_velocity.y']
    angular_z = df['field.angular_velocity.z']

    f = 300.0
elif "mat" in extension:
    # Assumes # https://ustc-flicar.github.io/calibration/#imu
    import scipy.io
    mat_data = scipy.io.loadmat(INPUT_FILE)
    
    time_column = mat_data['data_imu'][:,0]
    print(f"dt {time_column[5]-time_column[4]}")
    accel_x = mat_data['data_imu'][:,1]
    accel_y = mat_data['data_imu'][:,2]
    accel_z = mat_data['data_imu'][:,3]
    angular_x = mat_data['data_imu'][:,4]
    angular_y = mat_data['data_imu'][:,5]
    angular_z = mat_data['data_imu'][:,6]

    f = 400.0
else:
    print("not valid extension")
    sys.exit(1)

linear_acceleration_magnitude = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
print( np.count_nonzero(linear_acceleration_magnitude == 0))
print(f"time {np.max(time_column)-np.min(time_column)} mean {np.mean(linear_acceleration_magnitude)}")

"""
# Create the plot.
plt.figure(figsize=(10, 6))
plt.plot(time_column, linear_acceleration_magnitude, marker='o', linestyle='-')

# Add labels and title.
plt.xlabel("Time")
plt.ylabel("Linear Acceleration Magnitude")
plt.title("Linear Acceleration Magnitude over Time")
plt.grid(True)
plt.xticks(rotation=45)
plt.tight_layout()

# Save the plot to a file.
plt.savefig(f"linear_acceleration_magnitude_{INPUT_FILE}.png")
"""


import gtsam
from gtsam.utils.plot import plot_pose3
from gtsam.symbol_shorthand import B, V, X
from gtbook.display import show
from mpl_toolkits.mplot3d import Axes3D

def main_loop(runner, scenario, graph, initial, T):
    # The factor index for the estimation rate
    i = 0

    for k, t in enumerate(np.arange(0, T, dt)):
        # get measurements and add them to PIM
        # measuredOmega = runner.measuredAngularVelocity(t)
        # measuredAcc = runner.measuredSpecificForce(t)
        measuredAcc = [accel_x[k], accel_y[k], accel_z[k]]
        measuredOmega = [angular_x[k], angular_y[k], angular_z[k]]

        print(f"w {measuredOmega} a {measuredAcc}")

        ### This is where all the magic happens!
        pim.integrateMeasurement(measuredAcc, measuredOmega, dt)

        if (k + 1) % int(1 / (dt * 5)) == 0: # TODO parameter for number of frames in between
            # Create IMU factor every second.
            factor = gtsam.ImuFactor(X(i), V(i), X(i + 1), V(i + 1), B(0), pim)
            graph.push_back(factor)

            # We have created the binary constraint, so we clear out the preintegration values.
            pim.resetIntegration()

            # Get the true state which we will corrupt with some additive noise terms defined below
            actual_state_i = scenario.navState(t + dt)

            # These are additive noise terms.
            rotationNoise = gtsam.Rot3.Expmap(np.random.randn(3) * 0.1)
            translationNoise = gtsam.Point3(*np.random.randn(3) * 1)
            poseNoise = gtsam.Pose3(rotationNoise, translationNoise)

            noisy_state_i = gtsam.NavState(
                actual_state_i.pose().compose(poseNoise),
                actual_state_i.velocity() + np.random.randn(3) * 0.1)

            initial.insert(X(i + 1), noisy_state_i.pose())
            initial.insert(V(i + 1), noisy_state_i.velocity())
            i += 1

    return graph, initial


# T = 12  # The timespan of our trajectory.
# dt = 1e-2  # 100 Hz frequency

T = 12  # The timespan of our trajectory
dt = 1/f # 1/frequency
imu_scaling_bias_noise = 1
velocity = np.array([2, 0, 0])  # The velocity we wish to move at.

scenarios = {
    "zero_twist": (np.zeros(3), np.zeros(3)),  # Zero motion, stationary trajectory.
    "forward_motion": (np.zeros(3), velocity),  # Move forward in the x axis at 2 m/s.
    "loop": (np.array([0, -np.radians(30), 0]), velocity),  # A loop-de-loop trajectory.
    "sick": (np.array([np.radians(30), -np.radians(30), 0]), velocity)  # A spiral trajectory, "sick" in surfer slang.
}


def plot_scenario(scenario,
                  T,
                  dt,
                  title="IMU trajectory scenario",
                  fignum=0,
                  maxDim=5):
    for t in np.arange(0, T, dt):
        actualPose = scenario.pose(t)
        plot_pose3(fignum, actualPose, axis_length=0.3)

        translation = actualPose.translation()
        maxDim = max([max(np.abs(translation)), maxDim])
        ax = plt.gca()
        ax.set_xlim3d(-maxDim, maxDim)
        ax.set_ylim3d(-maxDim, maxDim)
        ax.set_zlim3d(-maxDim, maxDim)
        ax.set_title(title)

    plt.show()

def add_priors(scenario, graph, initial):
    # Noise models for
    priorNoise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1)
    velNoise = gtsam.noiseModel.Isotropic.Sigma(3, 0.1)

    initial_state = scenario.navState(0)
    graph.push_back(
        gtsam.PriorFactorPose3(X(0), initial_state.pose(), priorNoise))
    graph.push_back(
        gtsam.PriorFactorVector(V(0), initial_state.velocity(), velNoise))

    initial.insert(B(0), actualBias)
    initial.insert(X(0), initial_state.pose())
    initial.insert(V(0), initial_state.velocity())

    return graph, initial

def plot_trajectory(values: gtsam.Values,
                    title: str = "Estimated Trajectory",
                    fignum: int = 1,
                    show: bool = False):
    i = 0
    while values.exists(X(i)):
        pose_i = values.atPose3(X(i))
        plot_pose3(fignum, pose_i, 1)
        i += 1
    plt.title(title)

    gtsam.utils.plot.set_axes_equal(fignum)

    plt.ioff()

    if show:
        plt.show()


for idx, scenario_name in enumerate(scenarios.keys()):
    if "zero" not in scenario_name:
        continue
    scenario = gtsam.ConstantTwistScenario(*scenarios[scenario_name])
    

    #accBias = np.array([-0.3, 0.1, 0.2])
    #gyroBias = np.array([0.1, 0.3, -0.1])

    accBias = np.array([0.00010665186474984292, 0.00010665186474984292, 0.00010665186474984292 ]) * imu_scaling_bias_noise
    gyroBias = np.array([2.4962696453306656e-06, 2.4962696453306656e-06, 2.4962696453306656e-06 ]) * imu_scaling_bias_noise
    actualBias = gtsam.imuBias.ConstantBias(accBias, gyroBias)

    pim_params = gtsam.PreintegrationParams.MakeSharedU(9.81)

    # Some arbitrary noise sigmas
    #gyro_sigma = 1e-3
    #accel_sigma = 1e-3
    accel_sigma = 0.00221228340113903 * imu_scaling_bias_noise
    gyro_sigma = 6.975178963996091e-05 * imu_scaling_bias_noise
    I_3x3 = np.eye(3)
    pim_params.setGyroscopeCovariance(gyro_sigma**2 * I_3x3)
    pim_params.setAccelerometerCovariance(accel_sigma**2 * I_3x3)
    pim_params.setIntegrationCovariance(1e-7**2 * I_3x3)

    # Define the PreintegratedImuMeasurements object here.
    pim = gtsam.PreintegratedImuMeasurements(pim_params, actualBias)

    runner = gtsam.ScenarioRunner(scenario, pim_params, dt, actualBias)
    # plot_scenario(scenario, T, dt, fignum=6)
    plot_scenario(scenario, T, dt, fignum=idx + 1, title=scenario_name)

    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    graph, initial = add_priors(scenario, graph, initial)
    graph, initial = main_loop(runner, scenario, graph, initial, T)

    plot_trajectory(initial, title="Initial Trajectory", fignum=7, show=True)

    lm_params = gtsam.LevenbergMarquardtParams()
    lm_params.setVerbosityLM("SUMMARY")
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, lm_params)
    result = optimizer.optimize()
    # print(result)

    plot_trajectory(result, fignum=8, show=True)
