# AURO
# Run in order from src auro directory before running each scenario command
source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30 #TURTLEBOT3
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
source /usr/share/gazebo/setup.sh
colcon build --symlink-install
source ./install/local_setup.bash


# Scenarios
1. ros2 launch solution solution_nav2_launch.py num_robots:=3
2. ros2 launch solution solution_nav2_launch.py num_robots:=3 obstacles:=false
3. ros2 launch solution solution_nav2_launch.py num_robots:=3 sensor_noise:=true
4. ros2 launch solution solution_nav2_launch.py num_robots:=3 initial_pose_package:='solution' initial_pose_file:='config/initial_poses.yaml'
5. ros2 launch solution solution_nav2_launch.py num_robots:=3 zone_top_left:=false?