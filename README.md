# simple_chassis_controller

## Overview

This is controller package for the DynamicX final assessment (see Chinese [requirement](doc/requirement.md)) in the Season 2025. 
I remade it before the final assessment of Season 2026 began. 

**Keywords:** RoboMaster, ROS, ros_control, PID, Mecanum wheel, TF, Odometer.

### License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: cqincat<br />
Affiliation: [Dynamicx]()<br />
Maintainer: cqincat, 2891343933@qq.com**

The simple_chassis_controller package has been tested under [ROS] Noetic on respectively 18.04 and 20.04. This is
research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [rm_description](https://github.com/gdut-dynamic-x/rm_description)
- [rm_bringup](https://github.com/rm-controls/rm_bringup.git)
- [rm_config](https://github.com/rm-controls/rm_config.git)
- [rm_control](https://github.com/rm-controls/rm_control.git)
- controller_interface
- forward_command_controller
- hardware_interface
- pluginlib

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package
using
```bash
	cd catkin_workspace/src
	git clone git@github.com:CQin-Cat/hero_chassis_controller.git
    # git clone https://github.com/gdut-dynamic-x/simple_chassis_controller.git
	cd ../
	catkin build
```
## Usage

Control the chassis with:
```bash
	roslaunch hero_chassis_controller hero_chassis_controller.launch
```
## Config files

### Config folder /config

* **controllers.yaml**  Params of hero_chassis_controller and joint_state_controller.

### Config folder /cfg

* **PID.cfg**  Params of PID.

## Launch files

* **hero_chassis_controller.launch:** Hero chassis only simulation and chassis controller

## Bugs & Feature Requests

Please report bugs and request features using
the [Issue Tracker](https://github.com/gdut-dynamic-x/simple_chassis_controller/issues)
.

[ROS]: http://www.ros.org
