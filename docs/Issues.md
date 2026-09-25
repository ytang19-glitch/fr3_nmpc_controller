. It is a duplicated workspace plus missing Pinocchio runtime libraries required by libfranka 0.20.4. 
This also explains why fake simulation works while the real hardware path fails.

vertify the install of vital ros2 package
ros2 pkg prefix franka_fr3_moveit_config
ros2 pkg prefix franka_hardware
ros2 pkg prefix franka_bringup
