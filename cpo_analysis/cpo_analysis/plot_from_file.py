#!/usr/bin/env python

import csv
import os.path as osp
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
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

    projection = Proj(
        "+proj=etmerc +ellps=WGS84 +lat_0={0} +lon_0={1} +x_0=0 +y_0=0 +z_0={2} +k_0=1".format(proj_origin[0],
                                                                                               proj_origin[1],
                                                                                               proj_origin[2]))

    day_seconds = UNIX_GPS_OFFSET + LEAP_SECONDS + gps_day * 24 * 3600

    with open(gga_path, newline='') as resultfile:
        spamreader = csv.reader(resultfile, delimiter=',', quotechar='|')
        tmp = []
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
                tmp.append([timestamp, x, y, z, fix_type, long, lat])

    return np.array(tmp)


def main():

    plt.rc('axes', labelsize=12, titlesize=14)
    plt.rcParams["font.family"] = "serif"

    dataset = "feb15a"

    csv_dir = "./../results/"           # todo - change
    # csv_file = dataset + "_filter.csv"
    csv_file = dataset + "_batch.csv"       # todo - update
    positions = np.genfromtxt(osp.join(csv_dir, csv_file), delimiter=',', skip_header=3)
    interval = np.genfromtxt(osp.join(csv_dir, csv_file), delimiter=',', skip_header=1, max_rows=1)

    enu_origin = np.genfromtxt(osp.join(csv_dir, csv_file), delimiter=',', max_rows=1)

    # get start and end time of SWF data to get correct section from Doppler, ground truth
    start_time = safe_float(interval[0])
    end_time = safe_float(interval[1])
    # start_time = 0
    # end_time = 29999999999

    gt_dir = "./../data/gpgga/"             # todo
    gt_file = dataset + "_gga.ASC"
    if dataset == "nov25c2":
        day = 2133 * 7 + 3  # Nov.25/20
    elif dataset[:5] == "jan13":
        day = 2140 * 7 + 3  # Jan.13/21
    elif dataset[:5] == "feb10":
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
    fig1.subplots_adjust(left=0.10, right=0.97, bottom=0.10, top=0.92)
    plt.plot(r_gt[:, 1] - r_gt[0, 1], r_gt[:, 2] - r_gt[0, 2], label='GPS Ground Truth', c='C0', alpha=0.5)
    plt.plot(r_rtk[:, 1] - r_gt[0, 1], r_rtk[:, 2] - r_gt[0, 2], label='RTK Ground Truth', c='C0')

    plt.plot(positions[:, 1] - positions[0, 1], positions[:, 2] - positions[0, 2], label='Estimated', c='C1')

    plt.axis('equal')
    plt.title('Overhead View - {0}'.format(dataset))
    plt.xlabel('x (m)')
    plt.ylabel('y (m)')
    plt.legend()

    plt.show()


if __name__ == '__main__':
    main()
