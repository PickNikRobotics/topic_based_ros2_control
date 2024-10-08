# -*- coding: utf-8 -*-
import os
from pathlib import Path

import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

SCRIPT_PATH = Path(os.path.realpath(__file__)).parent


def generate_launch_description():
    ros2_controllers_file = Path(SCRIPT_PATH / "ros2_controllers.yaml")
    robot_description = {
        "robot_description": xacro.process_file(SCRIPT_PATH / "rrr.urdf.xacro").toxml(),
    }
    controllers = ["joint_state_broadcaster", "joint_trajectory_controller"]
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, ros2_controllers_file],
                output="screen",
            ),
        ]
        + [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
            for controller in controllers
        ],
    )
