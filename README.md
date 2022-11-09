# Isaac Ros2 Control

Description: This is a ROS 2 package for integrating the ros2_control with the [NVIDIA Isaac Sim](https://developer.nvidia.com/isaac-sim).


<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

[![Build Status](https://github.com/PickNikRobotics/isaac_ros2_control/actions/workflows/build_and_test.yaml/badge.svg)](https://github.com/PickNikRobotics/isaac_ros2_control/actions/workflows/build_and_test.yaml)

## Installation

### Build from Source

These instructions assume you are running on Ubuntu 20.04:

1. [Install ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). You can stop following along with the tutorial after you complete the section titled: [Environment setup](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#environment-setup). Make sure you setup your environment with:

        source /opt/ros/humble/setup.bash

   > *NOTE:* You may want to add that line to your `~/.bashrc`

2. [Install ROS2 Build Tools](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools)

   > *NOTE:* If installing on a fresh OS, run `sudo rosdep init` and `rosdep update` before the install script

3. Create a colcon workspace (*Note:* Feel free to change `~/ws_ros2` to whatever absolute path you want):

        export COLCON_WS=~/ws_ros2/
        mkdir -p $COLCON_WS/src

4. Get the repo:

        cd $COLCON_WS/src
        git clone git@github.com:PickNikRobotics/isaac_ros2_control.git

5. Download the required repositories and install any dependencies:

        cd $COLCON_WS/src
        vcs import < isaac_ros2_control/isaac_ros2_control.repos
        rosdep install --ignore-src --from-paths . -y

        # Pick a ROS_DOMAIN_ID that doesn't clash with others
        echo 'export ROS_DOMAIN_ID='<YOUR-NUMBER> >> ~/.bashrc

7. Configure and build the workspace:

        cd $COLCON_WS
        colcon build --symlink-install --event-handlers log-

8. Source the workspace.

        source $COLCON_WS/install/setup.bash

> *Note*: Whenever you open a new terminal be sure to run `source /opt/ros/humble/setup.bash`, `export COLCON_WS=~/ws_ros2`, and `source $COLCON_WS/install/setup.bash`. You may want to add those commands to your `~/.bashrc`

## For Developers

### Quickly update code repositories

To make sure you have the latest repos:

      cd $COLCON_WS/src/isaac_ros2_control
      git checkout main
      git pull origin main
      cd $COLCON_WS/src
      vcs import < isaac_ros2_control/isaac_ros2_control.repos
      rosdep install --from-paths . --ignore-src -y

### Setup pre-commit

pre-commit is a tool to automatically run formatting checks on each commit, which saves you from manually running clang-format (or, crucially, from forgetting to run them!).

Install pre-commit like this:

```
pip3 install pre-commit
```

Run this in the top directory of the repo to set up the git hooks:

```
pre-commit install
```

### Testing and Linting

To test the packages in isaac_ros2_control, use the following command with [colcon](https://colcon.readthedocs.io/en/released/).

    export TEST_PACKAGES="PROJECT_PACKAGE_NAMES"
    colcon build --packages-up-to ${TEST_PACKAGES}
    colcon test --packages-select ${TEST_PACKAGES}
    colcon test-result

To add a copyright for a new file

    ament_copyright --add-missing picknik bsd_3clause .

### Using ccache

ccache is a useful tool to speed up compilation times with GCC or any other sufficiently similar compiler.

To install ccache on Linux:

    sudo apt-get install ccache

For other OS, search the package manager or software store for ccache, or refer to the [ccache website](https://ccache.dev/)

#### Setup

To use ccache after installing it there are two methods. you can add it to your PATH or you can configure it for more specific uses.

ccache must be in front of your regular compiler or it won't be called. It is recommended that you add this line to your `.bashrc`:

    export PATH=/usr/lib/ccache:$PATH

To configure ccache for more particular uses, set the CC and CXX environment variables before invoking make, cmake, catkin_make or catkin build.

For more information visit the [ccache website](https://ccache.dev/).
