# Swarm-Project

Pull the Repository in ~/catkin_ws/src.

To Launch the Simulation for Meta-Heuristic Bat Algorithm with Heuristic Optimization
```
roslaunch bat_algo gazebo.launch
rosrun bat_algo detect_tanks.py
rosrun bat_algo publish_global_points.py
roslaunch bat_algo bats.launch
rosrun bat_algo set_params.py
roslaunch bat_algo go_to_point.launch
roslaunch bat_algo move_bat.launch
```

To Launch the Simulation for Pure Meta-Heuristic Bat Algorithm
```
roslaunch bat_algo gazebo.launch
roslaunch bat_algo bats.launch
rosrun bat_algo set_params.py
rosrun bat_algo pub_pose.py
roslaunch bat_algo go_to_point.launch
roslaunch bat_algo move_bat.launch
```

Realsense Plug In Source Repository: https://github.com/issaiass/realsense2_description
