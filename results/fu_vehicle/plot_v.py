import os
import pandas as pd
import sys
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_style('whitegrid')
plt.rcParams["figure.figsize"] = (15, 13)
point_size = 0.1

imu_no_gravity = pd.read_csv('../imu_no_gravity.csv')
imu_gravity = pd.read_csv('../imu_gravity.csv')

update_val = 1.5605008e9
start_val = 58 + update_val
end_val = 61.5 + update_val

#imu_no_gravity = imu_no_gravity[imu_no_gravity['timestamp'].le(end_val)]
#imu_no_gravity = imu_no_gravity[imu_no_gravity['timestamp'].ge(start_val)]

#imu_gravity = imu_gravity[imu_gravity['timestamp'].le(end_val)]
#imu_gravity = imu_gravity[imu_gravity['timestamp'].ge(start_val)]

var_x = imu_no_gravity.angular_velocity_x.var(ddof=0)
var_y = imu_no_gravity.angular_velocity_y.var(ddof=0)
var_z = imu_no_gravity.angular_velocity_z.var(ddof=0)
print(f'{var_x=}, {var_y=}, {var_z=}')
std_x = imu_no_gravity.angular_velocity_x.std(ddof=0)
std_y = imu_no_gravity.angular_velocity_y.std(ddof=0)
std_z = imu_no_gravity.angular_velocity_z.std(ddof=0)
print(f'{std_x=}, {std_y=}, {std_z=}')

ground_truth = pd.read_csv('../ground_truth_gps.csv')
odometry = pd.read_csv('vins_odom_.csv')
features = pd.read_csv('features_.csv')
bias = pd.read_csv('vins_bias_.csv')


entire_trajectory = True

if not entire_trajectory:
    # Drop data later than the last recorded odom
    ground_truth = ground_truth[ground_truth['timestamp'].le(odometry['timestamp'].iloc[-1])]

ground_truth['timestamp'] = ground_truth['timestamp'].astype(float)
odometry['timestamp'] = odometry['timestamp'].astype(float)

save_all = True
features_z_position_x_velocity = False
show_bias = False
xy_trajectory_from_above = False
compare_z = False
compare_velocities = False
compare_positions = False
z_vel_z_pos = False
rpy = False
plot_imu = True

if rpy or save_all:
    fig, ax = plt.subplots()
    ax.scatter(odometry['timestamp'],
               odometry['roll'], label='Estimated roll', linestyle='--', color='r',
               s=point_size, rasterized=True)
    ax.scatter(odometry['timestamp'],
               odometry['pitch'], label='Estimated pitch', linestyle='--', color='g', s=point_size, rasterized=True)
    ax.scatter(odometry['timestamp'],
               odometry['yaw'], label='Estimated yaw', linestyle='--', color='b', s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('Estimated orientation')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('degrees')
    fig.savefig('rpy_deg.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

"""if z_vel_z_pos or save_all:
    fig, axes = plt.subplots(nrows=2)
    axes[0].scatter(odometry['timestamp'],
                    odometry['z'], label='VINS z estimate', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[0].scatter(odometry['timestamp'],
                    odometry['vz'], label='VINS vz estimate', linestyle='--', color='darkred', s=point_size, rasterized=True)
    axes[0].legend()
    axes[0].set_title('Vins estimate z and vz')
    axes[0].set_xlabel('time [s]')

    axes[1].scatter(ground_truth['timestamp'],
                    ground_truth['z'], label='GPS z position', linestyle='--', color='g', s=point_size, rasterized=True)
    axes[1].scatter(ground_truth['timestamp'],
                    ground_truth['vz'], label='GPS z velocity', linestyle='--', color='lime', s=point_size, rasterized=True)
    axes[1].legend()
    axes[1].set_title('GPS z and vz')
    axes[1].set_xlabel('time [s]')
    fig.savefig('z_comparison_derivatives.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()"""
if show_bias or save_all:
    fig, axes = plt.subplots(nrows=3, ncols=1)
    axes[0].set_title('Estimated acc biases')
    axes[0].scatter(bias['timestamp'],
                    bias['acc_x'], label='bias acc_x', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[0].scatter(bias['timestamp'],
                    bias['acc_y'], label='bias acc_y', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[0].scatter(bias['timestamp'],
                    bias['acc_z'], label='bias acc_z', linestyle='--', color='g', s=point_size, rasterized=True)
    axes[0].legend()
    axes[0].set_xlabel('t [s]')
    axes[0].set_ylabel('a [m/s^2]')

    axes[1].set_title('Estimated gyro biases')
    axes[1].scatter(bias['timestamp'],
                    bias['gyro_x'], label='bias gyro_x', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[1].scatter(bias['timestamp'],
                    bias['gyro_y'], label='bias gyro_y', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[1].scatter(bias['timestamp'],
                    bias['gyro_z'], label='bias gyro_z', linestyle='--', color='g', s=point_size, rasterized=True)
    axes[1].legend()
    axes[1].set_xlabel('t [s]')
    axes[1].set_ylabel('v [rad/s]')

    axes[2].set_title('Some values for comparison')
    axes[2].scatter(odometry['timestamp'],
                    odometry['z'], label='Estimated Z position', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[2].scatter(ground_truth['timestamp'],
                    ground_truth['vx'], label='GPS X velocity', linestyle='--', color='black', s=point_size, rasterized=True)
    axes[2].scatter(odometry['timestamp'],
                    odometry['vy'], label='Estimated Y velocity', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[2].legend()
    axes[2].set_xlabel('t [s]')

    fig.savefig('biases.svg', format='svg', dpi=600)
    fig.tight_layout()
    # plt.show()
    plt.cla()

if features_z_position_x_velocity or save_all:
    fig, ax = plt.subplots()
    ax.scatter(features['timestamp'],
               features['tracked_features'], label='Matched features in sliding window', linestyle='--', color='b',
               s=point_size, rasterized=True)
    ax.scatter(odometry['timestamp'],
               odometry['z'], label='Estimated Z position', linestyle='--', color='r', s=point_size, rasterized=True)
    ax.scatter(ground_truth['timestamp'],
               ground_truth['vx'], label='GPS x velocity', linestyle='--', color='g', s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('Correspondence of matched features and Z estimate')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('n features, velocity [m/s], position [m]')
    fig.savefig('features_over_z.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

if xy_trajectory_from_above or save_all:
    fig, ax = plt.subplots()
    ax.scatter(ground_truth['x'],
               ground_truth['y'], label='GPS ground truth', linestyle='--', color='b', s=point_size, rasterized=True)
    ax.scatter(odometry['x'],
               odometry['y'], label='VINS estimate', linestyle='--', color='r', s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('Comparison of GPS ground truth and estimated xy coordinates')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    fig.savefig('xy_from_above.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

if compare_z or save_all:
    fig, ax = plt.subplots()
    ax.scatter(ground_truth['timestamp'],
               ground_truth['z'], label='GPS ground truth', linestyle='--', color='b', s=point_size, rasterized=True)
    ax.scatter(odometry['timestamp'],
               odometry['z'], label='VINS estimate', linestyle='--', color='r', s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('Comparison of GPS ground truth and estimated z coordinates')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('z [m]')
    fig.savefig('z_position_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

if compare_velocities or save_all:
    fig, axes = plt.subplots(nrows=3, ncols=1)
    axes[0].set_title('Comparison of GPS ground truth and estimated velocities')
    axes[0].scatter(ground_truth['timestamp'],
                    ground_truth['vx'], label='GPS x velocity', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[0].scatter(odometry['timestamp'],
                    odometry['vx'], label='VINS x velocity', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[0].legend()
    axes[0].set_xlabel('t [s]')
    axes[0].set_ylabel('v [m/s]')

    axes[1].scatter(ground_truth['timestamp'],
                    ground_truth['vy'], label='GPS y velocity', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[1].scatter(odometry['timestamp'],
                    odometry['vy'], label='VINS y velocity', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[1].legend()
    axes[1].set_xlabel('t [s]')
    axes[1].set_ylabel('v [m/s]')

    axes[2].scatter(ground_truth['timestamp'],
                    ground_truth['vz'], label='GPS z velocity', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[2].scatter(odometry['timestamp'],
                    odometry['vz'], label='VINS z velocity', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[2].legend()
    axes[2].set_xlabel('t [s]')
    axes[2].set_ylabel('v [m/s]')
    fig.savefig('velocities_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.cla()

if compare_positions or save_all:
    fig, axes = plt.subplots(nrows=3, ncols=1)
    axes[0].set_title('Comparison of GPS ground truth and estimated positions')
    axes[0].scatter(ground_truth['timestamp'],
                    ground_truth['x'], label='global GPS x coord', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[0].scatter(odometry['timestamp'],
                    odometry['x'], label='global VINS x coord', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[0].legend()
    axes[0].set_xlabel('t [s]')
    axes[0].set_ylabel('s [m]')

    axes[1].scatter(ground_truth['timestamp'],
                    ground_truth['y'], label='global GPS y coord', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[1].scatter(odometry['timestamp'],
                    odometry['y'], label='global VINS y coord', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[1].legend()
    axes[1].set_xlabel('t [s]')
    axes[1].set_ylabel('s [m]')

    axes[2].scatter(ground_truth['timestamp'],
                    ground_truth['z'], label='global GPS z coord', linestyle='--', color='b', s=point_size, rasterized=True)
    axes[2].scatter(odometry['timestamp'],
                    odometry['z'], label='global VINS z coord', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[2].legend()
    axes[2].set_xlabel('t [s]')
    axes[2].set_ylabel('s [m]')
    fig.savefig('position_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

if plot_imu:
    #imu_no_gravity = pd.read_csv('../imu_no_gravity.csv')
    fig, ax = plt.subplots(nrows=2)
    ax[0].scatter(imu_no_gravity['timestamp'],
                  imu_no_gravity['linear_acceleration_x'], label='lin acc x', linestyle='--', s=point_size, rasterized=True)
    ax[0].scatter(imu_no_gravity['timestamp'],
                  imu_no_gravity['linear_acceleration_y'], label='lin acc y', linestyle='--', s=point_size, rasterized=True)
    ax[0].scatter(imu_no_gravity['timestamp'],
                  imu_no_gravity['linear_acceleration_z'], label='lin acc z', linestyle='--', s=point_size, rasterized=True)
    ax[0].legend()
    ax[0].set_title('Linear accelerations without gravity')
    ax[0].set_xlabel('t [s]')
    ax[0].set_ylabel('m/s^2')

    #imu_gravity = pd.read_csv('../imu_gravity.csv')
    ax[1].scatter(imu_gravity['timestamp'],
                  imu_gravity['linear_acceleration_x'], label='lin acc x', linestyle='--', s=point_size, rasterized=True)
    ax[1].scatter(imu_gravity['timestamp'],
                  imu_gravity['linear_acceleration_y'], label='lin acc y', linestyle='--', s=point_size, rasterized=True)
    ax[1].scatter(imu_gravity['timestamp'],
                  imu_gravity['linear_acceleration_z'], label='lin acc z', linestyle='--', s=point_size, rasterized=True)
    ax[1].legend()
    ax[1].set_title('Linear accelerations with gravity')
    ax[1].set_xlabel('t [s]')
    ax[1].set_ylabel('m/s^2')
    fig.savefig('linear_accelerations_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

    fig, ax = plt.subplots(nrows=2)
    ax[0].scatter(imu_no_gravity['timestamp'],
                  imu_no_gravity['angular_velocity_x'], label='ang vel x', linestyle='--', s=point_size, rasterized=True)
    ax[0].scatter(imu_no_gravity['timestamp'],
                  imu_no_gravity['angular_velocity_y'], label='ang vel y', linestyle='--', s=point_size, rasterized=True)
    ax[0].scatter(imu_no_gravity['timestamp'],
                  imu_no_gravity['angular_velocity_z'], label='ang vel z', linestyle='--', s=point_size, rasterized=True)
    ax[0].legend()
    ax[0].set_title('Angular velocities without gravity')
    ax[0].set_xlabel('t [s]')
    ax[0].set_ylabel('rad/s')

    #imu_gravity = pd.read_csv('../imu_gravity.csv')
    ax[1].scatter(imu_gravity['timestamp'],
                  imu_gravity['angular_velocity_x'], label='ang vel x', linestyle='--', s=point_size, rasterized=True)
    ax[1].scatter(imu_gravity['timestamp'],
                  imu_gravity['angular_velocity_y'], label='ang vel y', linestyle='--', s=point_size, rasterized=True)
    ax[1].scatter(imu_gravity['timestamp'],
                  imu_gravity['angular_velocity_z'], label='ang vel z', linestyle='--', s=point_size, rasterized=True)
    ax[1].legend()
    ax[1].set_title('angular velocities with gravity')
    ax[1].set_xlabel('t [s]')
    ax[1].set_ylabel('rad/s')
    fig.savefig('angular_velocity_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()
