#!/bin/bash

# To stop control+C
trap "echo Exited!; exit;" SIGINT SIGTERM

# Loop indefinitely
while true; do
# First go to one limit of arm right 3
echo "Moving motors"
  ros2 topic pub /motor_controller_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''

joint_names: ['left_wheels', 'right_wheels']
points:
  -
    positions: [50, 50]
    time_from_start:
      sec: 5
      nanosec: 0 " --once

done
