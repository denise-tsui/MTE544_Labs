## NEW TERMINAL SOURCE / DOMAIN COMMANDS

source ~/robohub/turtlebot4/configs/.bashrc
export ROS_DOMAIN_ID=6 

## UNDOCK / TELEOP / DOCK COMMANDS

ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 action send_goal /dock irobot_create_msgs/action/Dock {}

## QOS PROFILE COMMAND

ros2 topic info /odom --verbose
