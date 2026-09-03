# Installation

## Binaries

`topic_based_ros2_control` is currently only released for ROS2 Humble and Rolling. For other ROS2 releases, you may need to build it from source.

        sudo apt-get install ros-$ROS_DISTRO-topic-based-ros2-control

## Build from Source

These instructions assume you are running on Ubuntu 22.04:

1. [Install ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html). You can stop following along with the tutorial after you complete the section titled: [Environment setup](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#environment-setup). Make sure you setup your environment with:

        source /opt/ros/humble/setup.bash

   > *NOTE:* You may want to add that line to your `~/.bashrc`

2. [Install ROS2 Build Tools](https://docs.ros.org/en/humble/Installation/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools)

   > *NOTE:* If installing on a fresh OS, run `sudo rosdep init` and `rosdep update` before the install script

3. Create a colcon workspace (*Note:* Feel free to change `~/ws_ros2` to whatever absolute path you want):

        export COLCON_WS=~/ws_ros2/
        mkdir -p $COLCON_WS/src

4. Get the repo and install any dependencies:

        cd $COLCON_WS/src
        git clone https://github.com/NVIDIA-ISAAC-ROS/topic_based_ros2_control.git

        rosdep install --ignore-src --from-paths . -y

        # Pick a ROS_DOMAIN_ID that doesn't clash with others
        # This one need to be the same value as TopicBased's domain id (0 by default)
        echo 'export ROS_DOMAIN_ID='<YOUR-NUMBER> >> ~/.bashrc

5. Configure and build the workspace:

        cd $COLCON_WS

   Isaac ROS on ROS 2 Lyrical uses Zenoh by default. Keep the middleware
   selection explicit when building and running a Lyrical workspace:

        export RMW_IMPLEMENTATION=rmw_zenoh_cpp

   Then build the workspace:

        colcon build --symlink-install --event-handlers log-

   Start a Zenoh router before launching ROS nodes:

        ros2 run rmw_zenoh_cpp rmw_zenohd

8. Source the workspace.

        source $COLCON_WS/install/setup.bash

> *Note*: Whenever you open a new terminal be sure to run `source /opt/ros/humble/setup.bash`, `export COLCON_WS=~/ws_ros2`, and `source $COLCON_WS/install/setup.bash`. You may want to add those commands to your `~/.bashrc`
