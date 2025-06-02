^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hsrb_robot_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2025-04-22)
-------------------
* Add a launch file for simultaneous node startup and fix the remap settings for the odom topic
* Enable velocity control for the base_roll_joint in the RViz simulator
* Fix tests of robot service
* Use common launch to bringup gazebo simulation
* Use common launch to bringup robot
* Move hsrb_rviz_simulator from hsrb_simulator
* Port hsrb_common_launch to ROS2
* Remove unnecessary parameters for navigation
* Change the package directory name from hsrb_servo_motor_protocol to tmc_exxx_servo_motor_protocol
* Modify exec_depend of hsrb_gazebo_launch
* Fix initial orientation
* add tmc_grid_map_server, add tf odom to map
* Contributors: Hiroaki Yaguchi, Keisuke Takeshita

2.0.0 (2024-10-16)
-------------------
* Initial release
* Contributors: Masayuki Masuda

