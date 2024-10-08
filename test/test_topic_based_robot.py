#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from topic_based_robot import TopicBasedRobot

# Tests
# 1- State initial_value != Command initial_value
# 2- Robot initial_value != State initial_value
# 3- Robot initial_value != Command initial_value
# 4- All of them are different
# 5- Launch this before the controllers + other way around
rclpy.init()

robot = TopicBasedRobot(["joint_1", "joint_2", "joint_3"])
# Values from initial_value from rrr.urdf.xacro
current_joint_state = robot.get_current_joint_state()
urdf_initial_values = [0.2, 0.3, 0.1]
assert (
    current_joint_state == urdf_initial_values
), f"{current_joint_state=} != {urdf_initial_values=}"

joint_state = [0.1, 0.2, 0.3]
robot.set_joint_positions(joint_state)
current_joint_state = robot.get_current_joint_state()
assert current_joint_state == joint_state, f"{current_joint_state=} != {joint_state=}"
