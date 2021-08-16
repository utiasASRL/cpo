#!/usr/bin/env python

import csv
import os.path as osp
import argparse
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


def read_gpgga(gga_path, gps_day, start_time=0.0, end_time=4999999999.9):
    """Read file of ASCII GPGGA messages and return measurements as array in UTM coordinates"""

    day_seconds = UNIX_GPS_OFFSET + gps_day * 24 * 3600

    proj_origin = (43.7822845, -79.4661581, 169.642048)  # hardcoding for now - todo: get from ground truth CSV
    projection = Proj(
        "+proj=etmerc +ellps=WGS84 +lat_0={0} +lon_0={1} +x_0=0 +y_0=0 +z_0={2} +k_0=1".format(proj_origin[0],
                                                                                               proj_origin[1],
                                                                                               proj_origin[2]))
    with open(gga_path, newline='') as result_file:
        reader = csv.reader(result_file, delimiter=',', quotechar='|')
        tmp = []
        distance_along_path = 0
        for i, row in enumerate(reader):
            if row[0] == "$GPGGA":
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
                        dist_added = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
                        distance_along_path += dist_added

                    tmp.append([timestamp, x, y, z, fix_type, long, lat, distance_along_path])
            elif row[0] == "$NAVSAT":
                lat = safe_float(row[3])
                long = safe_float(row[4])
                z = safe_float(row[5]) + 37.2       # todo: altitude measured differently in NavSatFix vs $GPGGA
                x, y = projection(long, lat)
                fix_type = 4 if safe_float(row[6]) < 0.2 else 1         # rough
                timestamp = safe_float(row[1]) + 1e-9 * safe_float(row[2])

                # TODO: is this fair? rounding down to 10th of a second for comparison
                timestamp = 0.1 * math.floor(timestamp * 10.0)

                if start_time <= timestamp <= end_time:
                    if len(tmp) > 0:
                        prev_x = tmp[-1][1]
                        prev_y = tmp[-1][2]
                        dist_added = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
                        distance_along_path += dist_added

                    tmp.append([timestamp, x, y, z, fix_type, long, lat, distance_along_path])

    return np.array(tmp)


def main():
    plt.rc('axes', labelsize=12, titlesize=14)
    plt.rcParams["font.family"] = "serif"

    datasets = ("feb10a", "feb10b", "feb10c", "feb10d",
                "feb15a", "feb15c", "feb15e", "feb15f",
                "aug61a", "aug61b", "aug61c", "aug61d")  # todo: don't actually have these files yet

    fig2, ax2 = plt.subplots(nrows=3, ncols=1, figsize=[8, 8])
    fig2.subplots_adjust(left=0.10, bottom=0.06, right=0.96, top=0.93)

    for i in range(len(datasets)):
        dataset = datasets[i]

        # GPS day required to parse ground truth. We determine it here.
        if dataset[:5] == "feb10":
            gt_dir = osp.expanduser("~/CLionProjects/ros2-ws/src/cpo_analysis/data/groundtruth/")
            gt_file = dataset + "_gga.ASC"
            day = 2144 * 7 + 3  # Feb.10/21
            trim_start_rows = 15  # optionally can be used to trim off part before robot begins driving
        elif dataset[:5] == "feb15":
            gt_dir = osp.expanduser("~/CLionProjects/ros2-ws/src/cpo_analysis/data/groundtruth/")
            gt_file = dataset + "_gga.ASC"
            day = 2145 * 7 + 1  # Feb.15/21
            trim_start_rows = 15
        else:
            gt_dir = osp.expanduser("~/ASRL/data/aug6/gt/")
            gt_file = dataset + ".csv"      # Aug.6/21 (navsatfix data)
            day = 0
            trim_start_rows = 150

        estimates_path = "~/Desktop/cpo_" + dataset + ".csv"
        estimates_path = osp.expanduser(estimates_path)
        estimates = np.genfromtxt(estimates_path, delimiter=',', skip_header=1 + trim_start_rows)

        # get start and end time of SWF data to get correct section from Doppler, ground truth
        start_time = safe_float(estimates[0, 0])
        end_time = safe_float(estimates[-1, 0])

        r_gt = read_gpgga(osp.join(gt_dir, gt_file), day, start_time=start_time, end_time=end_time)

        if not len(r_gt) > 0:
            raise ValueError('Ground truth between start and end time empty. Check if using correct ground truth file.')

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

        ax2[0].plot(relative_errors[:, 7] - relative_errors[0, 7], relative_errors[:, 4], c='C0')  # x errors
        ax2[1].plot(relative_errors[:, 7] - relative_errors[0, 7], relative_errors[:, 5], c='C0')  # y errors
        ax2[2].plot(relative_errors[:, 7] - relative_errors[0, 7],
                    np.sqrt(relative_errors[:, 4] ** 2 + relative_errors[:, 5] ** 2), c='C0')  # planar errors

    ax2[0].set_title('Position Errors wrt Ground Truth - All Runs')
    ax2[2].set_xlabel('Distance Along Path (m)')
    ax2[0].set_ylabel('x Error (m)')
    ax2[0].set_ylim([-1.6, 1.6])
    ax2[1].set_ylabel('y Error (m)')
    ax2[1].set_ylim([-1.6, 1.6])
    ax2[2].set_ylabel('2D Position Error (m)')
    ax2[2].set_ylim([0, 2])

    plt.show()


if __name__ == '__main__':
    main()
