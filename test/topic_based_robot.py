# -*- coding: utf-8 -*-

from collections import OrderedDict

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import JointState


class TopicBasedRobot(Node):
    """Topic Based Robot to test topic_based_ros2_control interface"""

    def __init__(self, joint_names: list[str]) -> None:
        super().__init__("topic_based_robot")
        self.joint_names = joint_names.copy()
        self.last_joint_command = []
        self.current_joint_state = []
        self.callback_group = ReentrantCallbackGroup()
        # Publisher for the robot internal joint state (Ground truth)
        self.actual_joint_state_publisher = self.create_publisher(
            JointState,
            "topic_based_joint_states",
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        # Subscriber for the desired joint state from the controller
        self.desired_joint_state_subscriber = self.create_subscription(
            JointState,
            "topic_based_joint_commands",
            self.command_callback,
            QoSProfile(depth=1),
            callback_group=self.callback_group,
        )
        # Reported joint state from ros2_control
        self.current_joint_state_subscriber = self.create_subscription(
            JointState,
            "joint_states",
            self.joint_states_callback,
            qos_profile_system_default,
            callback_group=self.callback_group,
        )

    def filter_joint_state_msg(self, msg: JointState):
        joint_states = []
        for joint_name in self.joint_names:
            try:
                index = msg.name.index(joint_name)
            except ValueError:
                msg = f"Joint name '{joint_name}' not in input keys {msg.name}"
                raise ValueError(msg) from None
            joint_states.append(msg.position[index])
        return joint_states

    def command_callback(self, msg: JointState):
        self.last_joint_command = self.filter_joint_state_msg(msg)

    def joint_states_callback(self, msg: JointState):
        self.current_joint_state = self.filter_joint_state_msg(msg)

    def get_current_joint_command(self) -> OrderedDict[str, float]:
        """Get the last joint command sent to the robot."""
        self.last_joint_command = []
        while len(self.last_joint_command) == 0:
            self.get_logger().warning(
                f"Waiting for joint command '{self.desired_joint_state_subscriber.topic_name}'...",
                throttle_duration_sec=2.0,
                skip_first=True,
            )
            rclpy.spin_once(self, timeout_sec=0.0)
        return self.last_joint_command

    def get_current_joint_state(self) -> OrderedDict[str, float]:
        """Get the current joint state reported by ros2_control on joint_states topic."""
        self.current_joint_state = []
        while len(self.current_joint_state) == 0:
            self.get_logger().warning(
                f"Waiting for current joint states from topic '{self.current_joint_state_subscriber.topic_name}'...",
                throttle_duration_sec=2.0,
                skip_first=True,
            )
            rclpy.spin_once(self, timeout_sec=0.0)
        return self.current_joint_state

    def set_joint_positions(
        self,
        joint_positions: list[float] | np.ndarray,
    ) -> None:
        """Set the joint positions of the robot."""
        self.actual_joint_state_publisher.publish(
            JointState(
                name=list(self.joint_names),
                position=joint_positions,
            ),
        )
        while not np.allclose(
            self.get_current_joint_state(),
            joint_positions,
            atol=1e-3,
        ):
            rclpy.spin_once(self, timeout_sec=0.0)