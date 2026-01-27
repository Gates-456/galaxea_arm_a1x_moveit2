reset && source ~/install/setup.bash && ros2 launch HDAS a1xy.py

reset && source ~/install/setup.bash && ros2 launch mobiman A1x_jointTrackerdemo_launch.py

reset && source ~/install/setup.bash && ros2 launch mobiman A1x_arm_relaxed_ik_launch.py




reset && ros2 topic echo /hdas/feedback_arm

reset && ros2 topic echo /motion_target/target_joint_state_arm

reset && ros2 topic echo /joint_states 




reset && source ~/arm/a1x/a1x_ws/install/setup.bash && colcon build

reset && source ~/arm/a1x/a1x_ws/install/setup.bash && ros2 launch a1x_hardware_bridge a1x_bridge.launch.py 

reset && source ~/arm/a1x/a1x_ws/install/setup.bash && ros2 launch a1x_moveit_config demo_with_hardware.launch.py 




robot@robot-GTi:~$ ros2 topic pub /motion_target/target_pose_arm geometry_msgs/msg/PoseStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: -0.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"



robot@robot-GTi:~$ ros2 topic pub /motion_target/target_joint_state_arm sensor_msgs/msg/JointState '
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ""
name: []
position: [0.0, 0.0, 0.0, 0.0, 0.0,0.0]
velocity: [0.1, 0.1, 0.1, 0.1, 0.1, 10.0]
effort: []'



