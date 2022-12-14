# User Guide

## Isaac Sim

Running Isaac with ROS2 requires setting the RMW implementation to fastdds ([see](https://forums.developer.nvidia.com/t/issues-running-ros2-humble-with-isaac-2022-1-1/229173)) with the following config file [rtps_udp_profile.xml](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docker/middleware_profiles/rtps_udp_profile.xml).

You need to export the following environment variables

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=path-to-rtps_udp_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
### MoveIt integration

* First we need to launch `omniverse-launcher-linux.AppImage` to start the `nucleus` server to be able to access Isaac Sim assets.

* Then we launch the demo by running `~/.local/share/ov/pkg/isaac_sim-2022.1.1/python.sh PATH-TO-CONFIG-DIR/isaac_moveit.py` and wait until isaac is fully loaded.

* Finally launch move_group with IsaacSystem `ros2 launch moveit_resources_panda_moveit_config demo.launch.py ros2_control_hardware_type:=isaac` and add MotionPlanning rviz plugin.

## Isaac System
The isaac system implements `hardware_interface::SystemInterface` supporting command and state interfaces.

### ros2_control urdf tag

The isaac system interface has a few `ros2_control` urdf tags to customize its behavior.

#### Parameters

* joint_commands_topic: (default: "/joint_command"). Example: `<param name="joint_commands_topic">/isaac_joint_commands</param>`.
* joint_states_topic: (default: "/isaac_joint_states"). Example: `<param name="joint_states_topic">/isaac_custom_joint_states</param>`.

#### Per-joint Parameters

* mimic: Defined name of the joint to mimic. This is often used concept with parallel grippers. Example: `<param name="mimic">joint1</param>`.
* multiplier: Multiplier of values for mimicking joint defined in mimic parameter. Example: `<param name="multiplier">-2</param>`.

### Modifying the urdf `ros2_control` tag for new robots

If your robot description support mock_components you only need to change `<plugin>mock_components/GenericSystem</plugin>` to `<plugin>isaac_ros2_control/IsaacSystem</plugin>`, make sure to add the `joint_commands_topic` and `joint_states_topic` to point to the correct Isaac sim topics.

```xml
        <ros2_control name="name" type="system">
            <hardware>
                <plugin>isaac_ros2_control/IsaacSystem</plugin>
                <param name="joint_commands_topic">/isaac_joint_commands</param>
                <param name="joint_states_topic">/isaac_joint_states</param>
            </hardware>
            <joint name="joint_1">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
            <joint name="joint_n">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
            <joint name="mimic_joint_1">
                <param name="mimic">joint_k</param>
                <param name="multiplier">1</param>
                <command_interface name="position" />
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
            <joint name="mimic_joint_n">
                <param name="mimic">joint_kn</param>
                <param name="multiplier">1</param>
                <command_interface name="position" />
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
```
