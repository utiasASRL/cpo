## Carrier Phase Odometry

### Overview
Carrier Phase Odometry (CPO) is a ROS2 project that uses raw GPS measurements from a single on-board receiver to estimate a robot's pose over time.
By using time-differenced carrier phase (TDCP) measurements, it can more accurately reconstruct the relative motion of a robot than simply differencing traditional positioning solutions.
TDCP borrows concepts from real-time kinetic (RTK) positioning but, importantly, does not require any additional receivers or communication links.

Together, these packages form a full state estimation pipeline with raw GPS measurements going in the front end and ROS2 Pose msgs coming out the back end.
Alternatively, you may want to use only the front end for preprocessing and combine the measurements with other sensors (e.g. visual odometry) in your own back end.

To date, the project has only been tested on Ubuntu 20.04 with a NovAtel Smart6L GPS receiver.
However, the code should be compatible with other versions of Linux and any receiver that can log over a serial port.

### Packages

#### `cpo_frontend`

This package acts as a driver and preprocessor for the carrier phase measurements.
The input is RTCM1004 (GPS observables) and RTCM1019 (GPS ephemerides) messages logged over serial.
These are standard [RTCMv3 messages](https://www.use-snip.com/kb/knowledge-base/rtcm-3-message-list/) that the vast majority of modern GPS receivers are capable of logging.
Instructions for configuring your receiver are [provided](#receiver-setup).
In the meantime, sample data is provided in `cpo_frontend/data/rtcm3` you may use to experiment with.
The packages support using ROS2's simulation time for offline testing.

The output of this package is a stream of TDCP msgs, defined in `cpo_interface`, published on the `/tdcp` topic.
These act as a pseudo-measurements pairing a set of satellites observed at two consecutive time points.
Corrections to the carrier phase measurements (such as estimating the tropospheric delay) are also handled in this package.

#### `cpo_backend`

The back end package handles the state estimation.
It subscribes to the `/tdcp` topic and publishes estimates on the `/cpo_enu` topic.
These estimates are a full SE(3) pose estimate of the vehicle in the East-North-Up frame.
The receiver position is converted to a vehicle pose via the fixed sensor-vehicle transform defined in the parameter file, and a nonholonomic motion model.
By default, a pose estimate is published each time we receive a TDCP msg.
Alternatively, if you set `fixed_rate_publish` to `true`, the estimated trajectory will be queried to give pose estimates at a fixed rate.
As this is an odometry algorithm, its goal is to achieve relative accuracy (i.e. where am I compared to where I was a minute ago).
Global accuracy (i.e. what is my longitude and latitude) is not improved by this method.

#### `cpo_interfaces`

This package defines 2 custom ROS2 msgs used as a pseudo-measurement to be passed between the front and back ends.

#### `cpo_analysis`

This package contains scripts to visualize and analyze the results of CPO.
`listener` subscribes to the estimate messages while CPO is running and plots the positions live.
`plot_file` is used to plot the integrated odometry afterwards as well as errors with respect to ground truth.

### Installation
- Install ROS2
  - CPO requires a ROS2 installation to overlay.
    The current release was developed and tested for the Foxy distribution.
    See the [ROS website](https://docs.ros.org/en/foxy/Installation.html) for installation instructions.
  - If you have not already installed [colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html), install it with:
  ```bash
  sudo apt install python3-colcon-common-extensions
  ```
- Install CPO and dependencies
  - `cpo_backend` uses [STEAM](https://github.com/utiasASRL/steam) as an engine for optimization and [lgmath](https://github.com/utiasASRL/lgmath) for handling Lie group operations.
    Following the instructions below will download and install the ROS2 versions of STEAM and lgmath.
    The default workspace folder is `~/cpo_workspace`.
    Note: the query trajectory service currently requires the `ros2-traj-covariance` branch of STEAM.
  - The dependencies of `cpo_frontend`, [RTKLIB](https://github.com/tomojitakasu/RTKLIB) and [serial](https://github.com/cottsay/serial) are included in the `deps` folder as submodules.
  ```bash
  # install system dependencies
  sudo apt install git tmux tmuxp python3-pip
  pip3 install scipy pyproj seaborn
  # source ROS2 workspace using either...
  source /opt/ros/foxy/setup.bash                             # if installed from binary packages
  source <path-to-ROS2-install>/ros_foxy/install/setup.bash   # if built from source 
  # download CPO and its dependencies
  mkdir -p ~/cpo_workspace/src && cd $_
  git clone https://github.com/utiasASRL/lgmath.git
  cd lgmath
  git checkout ros2-dev
  cd ~/cpo_workspace/src
  git clone https://github.com/utiasASRL/steam.git
  cd steam
  git checkout ros2-traj-covariance
  cd ~/cpo_workspace/src
  git clone https://github.com/ben-congram/cpo.git
  cd cpo
  git submodule update --init --remote --recursive
  # build and install serial
  mkdir -p ~/cpo_workspace/src/cpo/deps/serial/build && cd $_
  cmake ..
  sudo cmake --build . --target install  # will install to /usr/local/[lib,bin]
  export LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}  # put this in bashrc
  # build and install ROS2 packages
  cd ~/cpo_workspace
  colcon build --symlink-install
  source ~/cpo_workspace/install/setup.bash
  ```

### Offline Demo
Included in the repository is some sample GPS data that can be used to run the offline version of CPO.
The instructions below assume you have installed the package into `~/cpo_workspace`.
If you have chosen another directory you will need to change some paths in the parameter files.
To get started run:
```bash
source ~/cpo_workspace/install/setup.bash
cd ~/cpo_workspace/src/cpo/cpo_frontend/tmuxp
tmuxp load offline-cpo.yaml
```

When the data has finished, use `tmux kill-session` to exit.
You can then run `ros2 run cpo_analysis plot_file` to see the results of the CPO estimates compared to the RTK ground truth. 
A quick video of this demo can be seen [here](https://youtu.be/4nlUpjzyal8).

### Receiver Setup

We provide here receiver configuration instructions tested on a NovAtel SMART6-L receiver.
The exact commands required will vary for other receivers, but the strategy should be similar.
Please consult your receiver's manual before altering its configuration.

To interface with the `cpo_frontend` package, the receiver should log RTCM1004 and RTCM1019 messages over serial.
(Other, less commonly used, RTCMv3 messages may also work but have not been tested).
RTCM1004 provides the raw GPS measurements from all satellites seen by the receiver at a given time point (epoch).
It should be logged at a consistent rate (e.g. 1 Hz).
RTCM1019 messages provide the GPS satellite ephemerides (its trajectory through space).
Each message gives this info for one of the satellites seen.
They do not need to be logged as frequently (e.g. 0.2 Hz).
See [this page](https://www.use-snip.com/kb/knowledge-base/rtcm-3-message-list/) for more information on RTCM messages.

These messages are typically output by a fixed base station in RTK-positioning.
For the NovAtel this means we must use the `movingbasestation enable` command to properly output them from the (moving) robot receiver.
The instructions below configure this logging over the COM3 serial port (we often used COM1 and 2 for logging RTK ground truth during development), so you may adjust as needed for the COM you are using.
We find it easiest to send these instructions via the [Windows NovAtel Connect GUI](https://novatel.com/products/firmware-options-pc-software/novatel-connect) but you should also be able to use a serial emulator like [picocom](https://linux.die.net/man/8/picocom).
You may want to send an `freset` command (factory reset) before executing the instructions below but be careful as this will restore the default settings for all COMs.

```
com com3 57600 n 8 1 n off on
interfacemode com3 novatel novatel
movingbasestation enable
log com3 rtcm1004b ontime 1
log com3 rtcm1019b ontime 5
saveconfig
```

In our hardware setup, the serial connection from the GPS is routed to a DB9 to USB connector before being plugged directly into the laptop running CPO.
You may need to change the `port_path` parameter in `cpo_frontend` from the default `/dev/ttyUSB0` if you have other USB connections.