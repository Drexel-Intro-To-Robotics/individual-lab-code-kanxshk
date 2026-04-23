#!/usr/bin/env python3
"""
ME571 Lab 1 - Bag plotting utility (fixed for pandas 2.x / matplotlib)
Usage:  MPLBACKEND=Agg python3 plot_bag.py <bagfile.bag> <label>
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from bagpy import bagreader


def arr(series):
    """Convert a pandas Series to a plain numpy array (matplotlib-safe)."""
    return np.asarray(series)


def t_rel(df):
    return arr(df['Time']) - float(df['Time'].iloc[0])


def plot_trajectory(odom, label):
    x = arr(odom['pose.pose.position.x'])
    y = arr(odom['pose.pose.position.y'])
    plt.figure(figsize=(7, 7))
    plt.plot(x, y, linewidth=2, color='tab:blue')
    plt.scatter(x[0], y[0], color='green', s=80, label='Start', zorder=5)
    plt.scatter(x[-1], y[-1], color='red', s=80, label='End', zorder=5)
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title(f'{label.capitalize()} - Trajectory (Odometry)')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f'{label}_trajectory.png', dpi=200)
    plt.close()
    print(f'  saved {label}_trajectory.png')


def plot_odom_velocity(odom, label):
    t = t_rel(odom)
    vx = arr(odom['twist.twist.linear.x'])
    wz = arr(odom['twist.twist.angular.z'])
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axs[0].plot(t, vx, color='tab:blue')
    axs[0].set_ylabel('Linear vel x [m/s]')
    axs[0].set_title(f'{label.capitalize()} - Velocity from /odom')
    axs[0].grid(True)
    axs[1].plot(t, wz, color='tab:orange')
    axs[1].set_ylabel('Angular vel z [rad/s]')
    axs[1].set_xlabel('time [s]')
    axs[1].grid(True)
    plt.tight_layout()
    plt.savefig(f'{label}_odom_vel.png', dpi=200)
    plt.close()
    print(f'  saved {label}_odom_vel.png')


def plot_cmd_vel(cmd, label):
    if cmd is None or cmd.empty:
        print('  no /cmd_vel data, skipping cmd_vel plot')
        return
    t = t_rel(cmd)
    lx = arr(cmd['linear.x'])
    az = arr(cmd['angular.z'])
    fig, axs = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axs[0].plot(t, lx, color='tab:blue')
    axs[0].set_ylabel('Commanded linear.x [m/s]')
    axs[0].set_title(f'{label.capitalize()} - Commanded Velocity (/cmd_vel)')
    axs[0].grid(True)
    axs[1].plot(t, az, color='tab:orange')
    axs[1].set_ylabel('Commanded angular.z [rad/s]')
    axs[1].set_xlabel('time [s]')
    axs[1].grid(True)
    plt.tight_layout()
    plt.savefig(f'{label}_cmd_vel.png', dpi=200)
    plt.close()
    print(f'  saved {label}_cmd_vel.png')


def plot_imu_accel(imu, label):
    t = t_rel(imu)
    ax = arr(imu['linear_acceleration.x'])
    ay = arr(imu['linear_acceleration.y'])
    az = arr(imu['linear_acceleration.z'])
    plt.figure(figsize=(10, 5))
    plt.plot(t, ax, label='a_x')
    plt.plot(t, ay, label='a_y')
    plt.plot(t, az, label='a_z')
    plt.xlabel('time [s]')
    plt.ylabel('Acceleration [m/s^2]')
    plt.title(f'{label.capitalize()} - IMU Linear Acceleration')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'{label}_imu_accel.png', dpi=200)
    plt.close()
    print(f'  saved {label}_imu_accel.png')


def plot_imu_gyro(imu, label):
    t = t_rel(imu)
    wx = arr(imu['angular_velocity.x'])
    wy = arr(imu['angular_velocity.y'])
    wz = arr(imu['angular_velocity.z'])
    plt.figure(figsize=(10, 5))
    plt.plot(t, wx, label='w_x')
    plt.plot(t, wy, label='w_y')
    plt.plot(t, wz, label='w_z')
    plt.xlabel('time [s]')
    plt.ylabel('Angular velocity [rad/s]')
    plt.title(f'{label.capitalize()} - IMU Angular Velocity')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'{label}_imu_gyro.png', dpi=200)
    plt.close()
    print(f'  saved {label}_imu_gyro.png')


def safe_read(bag, topic):
    try:
        csv_path = bag.message_by_topic(topic)
        if not csv_path or not os.path.exists(csv_path):
            return None
        df = pd.read_csv(csv_path)
        return df if not df.empty else None
    except Exception as e:
        print(f'  could not read {topic}: {e}')
        return None


def main():
    if len(sys.argv) < 3:
        print('Usage: python3 plot_bag.py <bagfile.bag> <label>')
        sys.exit(1)

    bagfile = sys.argv[1]
    label = sys.argv[2]

    if not os.path.exists(bagfile):
        print(f'Bag file not found: {bagfile}')
        sys.exit(1)

    print(f'Reading {bagfile} ...')
    bag = bagreader(bagfile)
    print('Topics in bag:')
    print(bag.topic_table)

    odom = safe_read(bag, '/odom')
    imu  = safe_read(bag, '/imu')
    cmd  = safe_read(bag, '/cmd_vel')

    if odom is None:
        print('ERROR: no /odom data in bag.')
        sys.exit(1)
    if imu is None:
        print('ERROR: no /imu data in bag.')
        sys.exit(1)

    print(f'Generating plots for label "{label}" ...')
    plot_trajectory(odom, label)
    plot_odom_velocity(odom, label)
    plot_cmd_vel(cmd, label)
    plot_imu_accel(imu, label)
    plot_imu_gyro(imu, label)
    print('Done.')


if __name__ == '__main__':
    main()