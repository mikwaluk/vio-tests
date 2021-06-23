import os
import pandas as pd
import sys
import matplotlib.pyplot as plt
import seaborn as sns

sns.set_style('whitegrid')
plt.rcParams["figure.figsize"] = (15, 13)
point_size = 0.1

ground_truth = pd.read_csv('../twist_gps_odom.csv')
odometry = pd.read_csv('vins_odom_normal_speed_test.csv')
features = pd.read_csv('features_normal_speed_test.csv')
bias = pd.read_csv('vins_bias_normal_speed_test.csv')


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
rpy = True

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
               features['features_0_4'], label='Matched features in sliding window', linestyle='--', color='b',
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
