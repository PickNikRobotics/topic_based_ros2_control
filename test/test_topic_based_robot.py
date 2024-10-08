#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from topic_based_robot import TopicBasedRobot

rclpy.init()

robot = TopicBasedRobot(["joint_1", "joint_2", "joint_3"])
# By default the joint_states should have the values from initial_value from rrr.urdf.xacro
current_joint_state = robot.get_current_joint_state()
urdf_initial_values = [0.2, 0.3, 0.1]
assert (
    current_joint_state == urdf_initial_values
), f"{current_joint_state=} != {urdf_initial_values=}"

# Test setting the robot joint states
joint_state = [0.1, 0.2, 0.3]
robot.set_joint_positions(joint_state)
current_joint_state = robot.get_current_joint_state()
assert current_joint_state == joint_state, f"{current_joint_state=} != {joint_state=}"
