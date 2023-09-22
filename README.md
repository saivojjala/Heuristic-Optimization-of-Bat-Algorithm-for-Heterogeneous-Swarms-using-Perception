# Heuristic Optimization of Bat Algorithm using Perception

This is the official code base for the publication [Heuristic Optimization of Bat Algorithm using Perception](https://oresta.org/article-view/?id=572) published in Operational Research in Engineering Sciences:
Theory and Applications Journal

## Graphical Abstract
![Graphical Abstract_Swarm Paper](https://github.com/saivojjala/Heuristic-Optimization-of-Bat-Algorithm-using-Perception/assets/75236655/895c9c99-5088-4ad6-91e9-355bbd0bf2ed)

To view the Simulation clone the repository into a ROS workspace using the following commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/saivojjala/Heuristic-Optimization-of-Bat-Algorithm-using-Perception.git
```

To Launch the Simulation for Meta-Heuristic Bat Algorithm with Heuristic Optimization:
```
roslaunch bat_algo gazebo.launch
rosrun bat_algo detect_tanks.py
rosrun bat_algo publish_global_points.py
roslaunch bat_algo bats.launch
rosrun bat_algo set_params.py
roslaunch bat_algo go_to_point.launch
roslaunch bat_algo move_bat.launch
```

To Launch the Simulation for Pure Meta-Heuristic Bat Algorithm:
```
roslaunch bat_algo gazebo.launch
roslaunch bat_algo bats.launch
rosrun bat_algo set_params.py
rosrun bat_algo pub_pose.py
roslaunch bat_algo go_to_point.launch
roslaunch bat_algo move_bat.launch
```

Realsense Plug In Source Repository: https://github.com/issaiass/realsense2_description


![ROS Package Index](https://img.shields.io/ros/v/noetic/std_msgs)
