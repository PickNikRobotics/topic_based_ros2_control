^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topic_based_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2023-06-09)
------------------
* colcon: fixup missing test dependency (`#9 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/9>`_)
  fixes ros2_control_test_assets .hpp file was not found
  Co-authored-by: Jafar Uruç <jafar.uruc@gmail.com>
* Contributors: Alex Moriarty

0.1.0 (2023-05-23)
------------------
* Fix joint state for mimic joints & expose epsilon parameter (`#7 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/7>`_)
  * Check the difference between command and current joint state to stop command publication
  * Set mimic joints values
  * Move magic numbers to constants
  * Remove last_position_command\_ variable
* Replace Isaac with TopicBased and generalize package (`#5 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/5>`_)
* Clean up documentation (`#4 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/4>`_)
* Removed unused node. (`#3 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/3>`_)
* Add issac ros2 control hardware interface (`#1 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/1>`_)
* Contributors: Alex Moriarty, Giovanni, Jafar Uruç
