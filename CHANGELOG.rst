^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package topic_based_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2025-08-16)
------------------
* Deprecation notices should be seen. Do not convert warnings into errors. (`#36 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/36>`_)
* Add integration test (`#28 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/28>`_)
  * Add ruff/ruff format to pre-commit & Add integration test
  * Set joint_states\_ to initial_value as well
* Update HARDWARE_INTERFACE_VERSION_GTE from hardware_interface's version.h (`#32 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/32>`_)
* Update README.md
* Fix RessourceManager constructor (`#30 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/30>`_)
  * Fix rm() constructor
  * Enable CI on jazzy/rolling
  * Add macro to check hardware_interface version
  ---------
  Co-authored-by: JafarAbdi <jafar.uruc@gmail.com>
* Update README.md (`#31 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/31>`_)
* Allow publisher to always publish by setting the threshold to zero. (`#22 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/22>`_)
  * Allow publisher to always publish by setting the threshold to zero.
  On documentation (https://github.com/PickNikRobotics/topic_based_ros2_control/blob/main/doc/user.md)
  trigger_joint_command_threshold can be configuration with zero so that the allow publisher to
  always send the joint command.
  However, in the code the condition was set with:
  ```
  if (diff <= trigger_joint_command_threshold\_)
  {
  return hardware_interface::return_type::OK;
  }
  ```
  In case trigger_joint_command_threshold\_ is set to zero, diff can still be zero so
  it will not trigger the send.
  This PR tries to address the above mentioned case.
  * Revert "Allow publisher to always publish by setting the threshold to zero."
  This reverts commit a799b539d877d1a2ca7203f5d57bc4c805df7202.
  * Update the documentation instead (-1).
* Contributors: Bence Magyar, Christoph Fröhlich, Jafar Uruç, Yong Tang

0.2.0 (2023-09-04)
------------------
* Check the mimicked joint interface before setting them (`#13 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/13>`_)
* Check ros context is valid before spinning the node (`#13 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/13>`_)
* Enable mobile base in Isaac (`#12 <https://github.com/PickNikRobotics/topic_based_ros2_control/issues/12>`_)
* Contributors: Jafar Uruç, Marq Rasmussen

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
