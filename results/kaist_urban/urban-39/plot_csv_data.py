import os
import pandas as pd
import sys
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.spatial.transform import Rotation
import numpy as np
sns.set_style('whitegrid')
plt.rcParams["figure.figsize"] = (15, 13)
point_size = 0.2

# imu_gravity = pd.read_csv('../imu_vi_car_bag.csv')


ground_truth_pos = pd.read_csv('../global_pose.csv')
odometry = pd.read_csv('vins_odom.csv')
features = pd.read_csv('features.csv')
bias = pd.read_csv('vins_bias.csv')
ground_truth_pos['timestamp'] = ground_truth_pos['timestamp'].astype(float) / 1e9

entire_trajectory = False

if not entire_trajectory:
    # Drop data later than the last recorded odom
    ground_truth_pos = ground_truth_pos[ground_truth_pos['timestamp'].le(odometry['timestamp'].iloc[-1])]
# Translate and rotate the ground truth to match the starting direction of the algorithm
initial_rotation = ground_truth_pos.iloc[0]
rot: Rotation = Rotation.from_matrix([[initial_rotation.R00, initial_rotation.R01, initial_rotation.R02],
                                      [initial_rotation.R10, initial_rotation.R11, initial_rotation.R12],
                                      [initial_rotation.R20, initial_rotation.R21, initial_rotation.R22]])
ground_truth_pos['x'] = ground_truth_pos['x'] - ground_truth_pos['x'][0]
ground_truth_pos['y'] = ground_truth_pos['y'] - ground_truth_pos['y'][0]
ground_truth_pos['z'] = ground_truth_pos['z'] - ground_truth_pos['z'][0]
R = Rotation.from_matrix(rot.as_matrix().transpose())
translated = R.apply(ground_truth_pos[['x', 'y', 'z']])
new_df = pd.DataFrame(translated, columns=['x', 'y', 'z'])
ground_truth_pos[['x', 'y', 'z']] = new_df[['x', 'y', 'z']]

save_all = True
features_z_position_x_velocity = False
show_bias = False
xy_trajectory_from_above = True
compare_z = False
compare_velocities = False
compare_positions = True
z_vel_z_pos = True
rpy = False
plot_imu = False

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
                    odometry['z'], label='Estimated z position', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[2].plot(ground_truth_pos['timestamp'],
                 ground_truth_pos['z'], label='Ground truth z position', color='b', rasterized=True)
    axes[2].legend()
    axes[2].set_xlabel('t [s]')

    fig.savefig('biases.svg', format='svg', dpi=600)
    fig.tight_layout()
    # plt.show()
    plt.cla()

if features_z_position_x_velocity or save_all:
    fig, ax = plt.subplots()
    ax.scatter(features['timestamp'],
               features['tracked_features'], label='Tracked features', linestyle='--', color='b',
               s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('Number of features tracked in the sliding window')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('n features, velocity [m/s], position [m]')
    fig.savefig('features.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

if xy_trajectory_from_above or save_all:
    fig, ax = plt.subplots()
    ax.scatter(ground_truth_pos['x'],
               ground_truth_pos['y'], label='Pseudo ground truth', color='b', s=point_size, rasterized=True)
    ax.scatter(odometry['x'],
               odometry['y'], label='VINS estimate', linestyle='--', color='r', s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('Comparison of the pseudo ground truth and estimated xy coordinates')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    fig.savefig('xy_from_above.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()
# pd.set_option("display.max_rows", None, "display.max_columns", None)
if compare_z or save_all:
    fig, ax = plt.subplots()
    ax.plot(ground_truth_pos['timestamp'],
            ground_truth_pos['z'], label='Pseudo ground truth', color='b', rasterized=True)
    ax.scatter(odometry['timestamp'],
               odometry['z'], label='VINS estimate', linestyle='--', color='r', s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('Comparison of pseudo ground truth and estimated z coordinates')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('z [m]')
    fig.savefig('z_position_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

if compare_velocities or save_all:
    fig, axes = plt.subplots(nrows=3, ncols=1)
    axes[0].set_title('Comparison of estimated velocities')
    axes[0].scatter(odometry['timestamp'],
                    odometry['vx'], label='VINS x velocity', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[0].legend()
    axes[0].set_xlabel('t [s]')
    axes[0].set_ylabel('v [m/s]')

    axes[1].scatter(odometry['timestamp'],
                    odometry['vy'], label='VINS y velocity', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[1].legend()
    axes[1].set_xlabel('t [s]')
    axes[1].set_ylabel('v [m/s]')

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
    axes[0].set_title('Comparison of pseudo ground truth and estimated positions')
    axes[0].plot(ground_truth_pos['timestamp'],
                 ground_truth_pos['x'], label='global pseudo ground truth x coord', color='b', rasterized=True)
    axes[0].scatter(odometry['timestamp'],
                    odometry['x'], label='global VINS x coord', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[0].legend()
    axes[0].set_xlabel('t [s]')
    axes[0].set_ylabel('s [m]')

    axes[1].plot(ground_truth_pos['timestamp'],
                 ground_truth_pos['y'], label='global pseudo ground truth y coord', color='b', rasterized=True)
    axes[1].scatter(odometry['timestamp'],
                    odometry['y'], label='global VINS y coord', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[1].legend()
    axes[1].set_xlabel('t [s]')
    axes[1].set_ylabel('s [m]')

    axes[2].plot(ground_truth_pos['timestamp'],
                 ground_truth_pos['z'], label='global pseudo gronud truth z coord', color='b', rasterized=True)
    axes[2].scatter(odometry['timestamp'],
                    odometry['z'], label='global VINS z coord', linestyle='--', color='r', s=point_size, rasterized=True)
    axes[2].legend()
    axes[2].set_xlabel('t [s]')
    axes[2].set_ylabel('s [m]')
    fig.savefig('position_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()

if plot_imu:
    # imu_no_gravity = pd.read_csv('../imu_no_gravity.csv')
    fig, ax = plt.subplots()
    print(imu_gravity)
    ax.scatter(imu_gravity['timestamp'],
               imu_gravity['linear_acceleration_x'], label='lin acc x', linestyle='--', s=point_size, rasterized=True)
    ax.scatter(imu_gravity['timestamp'],
               imu_gravity['linear_acceleration_y'], label='lin acc y', linestyle='--', s=point_size, rasterized=True)
    ax.scatter(imu_gravity['timestamp'],
               imu_gravity['linear_acceleration_z'], label='lin acc z', linestyle='--', s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('Linear accelerations with gravity')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('m/s^2')
    fig.savefig('linear_accelerations_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()
    fig, ax = plt.subplots()
    ax.scatter(imu_gravity['timestamp'],
               imu_gravity['angular_velocity_x'], label='ang vel x', linestyle='--', s=point_size, rasterized=True)
    ax.scatter(imu_gravity['timestamp'],
               imu_gravity['angular_velocity_y'], label='ang vel y', linestyle='--', s=point_size, rasterized=True)
    ax.scatter(imu_gravity['timestamp'],
               imu_gravity['angular_velocity_z'], label='ang vel z', linestyle='--', s=point_size, rasterized=True)
    ax.legend()
    ax.set_title('angular velocities with gravity')
    ax.set_xlabel('t [s]')
    ax.set_ylabel('rad/s')
    fig.savefig('angular_velocity_comparison.svg', format='svg', dpi=600)
    # plt.show()
    plt.clf()
