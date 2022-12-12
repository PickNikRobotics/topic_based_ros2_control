# User Guide

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
