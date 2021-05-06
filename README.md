## Carrier Phase Odometry

### Overview
todo

### Packages
todo

#### `cpo_frontend`


#### `cpo_backend`


#### `cpo_interfaces`


#### `cpo_analysis`


### Installation
- Install dependencies
  - CPO requires a ROS2 installation to overlay.
    The current release was developed and tested for the Foxy distribution.
    See the [ROS website](https://docs.ros.org/en/foxy/Installation.html) for installation instructions.
  - `cpo_backend` uses [STEAM](https://github.com/utiasASRL/steam) as an engine for optimization.
    Please follow the [instructions to install using CMake](https://github.com/utiasASRL/steam/blob/develop/INSTALL.md) to install the STEAM library and its dependencies.
  - The dependencies of `cpo_frontend`, [RTKLIB](https://github.com/tomojitakasu/RTKLIB) and [serial](https://github.com/cottsay/serial) are included in the `deps` folder.
- Install CPO
  ```bash
  source ~/<path-to-ROS2-install>/ros_foxy/install/setup.bash
  mkdir -p ~/<cpo_workspace> && cd ~/<cpo_workspace>
  git clone https://github.com/ben-congram/cpo.git
  colcon build --symlink-install
  source ~/<cpo_workspace>/install/setup.bash
  ```


todo - test instructions

### Receiver Setup
todo