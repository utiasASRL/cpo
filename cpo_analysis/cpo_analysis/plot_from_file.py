#!/usr/bin/env python

import csv
import os.path as osp

import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from math import sqrt
from pyproj import Proj
import seaborn as sns

sns.set_style("whitegrid")

matplotlib.use("TkAgg")  # Can change to 'Agg' for non-interactive mode
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42

# difference between starts of Unix time (Jan.1/70) and GPS time (Jan.6/80)
UNIX_GPS_OFFSET = 315964800
LEAP_SECONDS = 18


def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float('NaN')


def safe_int(field):
    try:
        return int(field)
    except ValueError:
        return 0


def read_gpgga(gga_path, gps_day, proj_origin, start_time=0.0, end_time=4999999999.9):
    """Read file of ASCII GPGGA messages and return measurements as array in UTM coordinates"""

    # todo: transverse mercator projection very slightly different from Euclidean ENU
    projection = Proj(
        "+proj=etmerc +ellps=WGS84 +lat_0={0} +lon_0={1} +x_0=0 +y_0=0 +z_0={2} +k_0=1".format(proj_origin[0],
                                                                                               proj_origin[1],
                                                                                               proj_origin[2]))

    day_seconds = UNIX_GPS_OFFSET + gps_day * 24 * 3600

    with open(gga_path, newline='') as resultfile:
        spamreader = csv.reader(resultfile, delimiter=',', quotechar='|')
        tmp = []
        distance_along_path = 0
        for i, row in enumerate(spamreader):
            if row[0] != "$GPGGA":
                continue

            lat_tmp = row[2]
            lat = safe_float(lat_tmp[0:2]) + safe_float(lat_tmp[2:]) / 60.0
            long_tmp = row[4]
            long = safe_float(long_tmp[0:3]) + safe_float(long_tmp[3:]) / 60.0
            if row[5] == 'W':
                long = -long
            z = safe_float(row[9])
            x, y = projection(long, lat)
            fix_type = safe_int(row[6])
            time_of_day = row[1]
            timestamp = day_seconds + safe_float(time_of_day[0:2]) * 3600.0 + safe_float(
                time_of_day[2:4]) * 60.0 + safe_float(time_of_day[4:])

            if start_time <= timestamp <= end_time:
                if len(tmp) > 0:
                    prev_x = tmp[-1][1]
                    prev_y = tmp[-1][2]
                    dist_added = sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
                    distance_along_path += dist_added

                tmp.append([timestamp, x, y, z, fix_type, long, lat, distance_along_path])

    return np.array(tmp)


def main():
    plt.rc('axes', labelsize=12, titlesize=14)
    plt.rcParams["font.family"] = "serif"

    dataset = "feb15c"  # todo: this should update automatically
    trim_start_rows = 10

    csv_dir = "/home/ben/CLionProjects/ros2-ws/src/cpo_analysis/data/estimates/"  # todo: non-hard-coded-path
    csv_file = "cpo.csv"
    enu_origin = np.genfromtxt(osp.join(csv_dir, csv_file), delimiter=',', max_rows=1)
    estimates = np.genfromtxt(osp.join(csv_dir, csv_file), delimiter=',', skip_header=1 + trim_start_rows)

    # get start and end time of SWF data to get correct section from Doppler, ground truth
    start_time = safe_float(estimates[0, 0])
    end_time = safe_float(estimates[-1, 0])

    gt_dir = "/home/ben/CLionProjects/ros2-ws/src/cpo_analysis/data/groundtruth/"
    gt_file = dataset + "_gga.ASC"
    if dataset[:5] == "feb10":
        day = 2144 * 7 + 3  # Feb.10/21
    elif dataset[:5] == "feb15":
        day = 2145 * 7 + 1  # Feb.15/21
    else:
        raise Exception("Unknown dataset - {0}".format(dataset))

    r_gt = read_gpgga(osp.join(gt_dir, gt_file), day, enu_origin, start_time=start_time, end_time=end_time)

    # extract portion of GPS that had RTK-Fixed fix (the gold standard)
    r_rtk = r_gt[r_gt[:, 4] == 4]

    # overhead plot
    fig1 = plt.figure(1, figsize=[9, 4.5])
    fig1.subplots_adjust(left=0.10, bottom=0.10, right=0.97, top=0.92)
    # plt.plot(r_gt[:, 1] - r_gt[0, 1], r_gt[:, 2] - r_gt[0, 2], label='GPS Ground Truth', c='C0', alpha=0.5)
    plt.plot(r_rtk[:, 1] - r_gt[0, 1], r_rtk[:, 2] - r_gt[0, 2], label='RTK Ground Truth', c='C0')

    plt.plot(estimates[:, 2] - estimates[0, 2], estimates[:, 3] - estimates[0, 3], label='Estimated', c='C1')

    plt.axis('equal')
    plt.title('Overhead View - {0} Dataset'.format(dataset))
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    plt.legend()

    # error plot
    tmp = []
    for row in r_gt:
        idx_np = np.where(estimates[:, 0] == row[0])
        if idx_np[0].size != 0:
            idx = safe_int(idx_np[0][0])
            tmp.append([estimates[idx, 0],  # GPS ref. timestamp
                        row[1],  # ground truth x (down-sampled)
                        row[2],  # "" y
                        row[3],  # "" z
                        (estimates[idx, 2] - estimates[0, 2]) - (row[1] - r_gt[0, 1]),  # estimator error x
                        (estimates[idx, 3] - estimates[0, 3]) - (row[2] - r_gt[0, 2]),  # "" y
                        (estimates[idx, 4] - estimates[0, 4]) - (row[3] - r_gt[0, 3]),  # "" z
                        row[7],  # distance along path
                        ])
    relative_errors = np.array(tmp)

    fig2, ax2 = plt.subplots(nrows=3, ncols=1, figsize=[8, 8])
    fig2.subplots_adjust(left=0.10, bottom=0.06, right=0.96, top=0.93)
    ax2[0].plot(relative_errors[:, 7] - relative_errors[0, 7], relative_errors[:, 4], c='C0')  # x errors
    ax2[1].plot(relative_errors[:, 7] - relative_errors[0, 7], relative_errors[:, 5], c='C0')  # y errors
    ax2[2].plot(relative_errors[:, 7] - relative_errors[0, 7],
                np.sqrt(relative_errors[:, 4] ** 2 + relative_errors[:, 5] ** 2), c='C0')  # planar errors

    ax2[0].set_title('Position Errors wrt Ground Truth - {0}'.format(dataset))
    ax2[2].set_xlabel('Distance Along Path (m)')
    ax2[0].set_ylabel('x Error (m)')
    ax2[0].set_ylim([-1.6, 1.6])
    ax2[1].set_ylabel('y Error (m)')
    ax2[1].set_ylim([-1.6, 1.6])
    ax2[2].set_ylabel('2D Position Error (m)')
    ax2[2].set_ylim([0, 2])

    # errors by time
    fig3 = plt.figure(3, figsize=[8, 3])
    plt.plot(relative_errors[:, 0] - 1613400000, np.sqrt(relative_errors[:, 4] ** 2 + relative_errors[:, 5] ** 2),
             c='C1')
    ax2[0].set_title('Position Errors by Timestamp - {0}'.format(dataset))
    plt.xlabel('Timestamp - 1613400000 (s)')
    plt.ylabel('2D Position Error (m)')
    plt.ylim([0, 2])
    plt.tight_layout()

    fig4 = plt.figure(4)
    tmp = []
    for row in estimates:
        if row[0] < 1613419716 or row[0] > 1613419897:
            continue
        yaw = -math.atan2(row[9], row[8])
        pitch = -math.atan2(-row[10], math.sqrt(row[14] ** 2 + row[18] ** 2))
        roll = -math.atan2(row[14], row[18])
        tmp.append([yaw, pitch, roll])
    ypr = np.array(tmp)
    plt.title("Standalone Carrier Phase Odometry")
    plt.xlabel("Seconds")
    plt.ylabel("Estimated Angle (rad)")
    plt.plot(ypr[:])  # plot yaw pitch roll

    plt.show()


if __name__ == '__main__':
    main()
