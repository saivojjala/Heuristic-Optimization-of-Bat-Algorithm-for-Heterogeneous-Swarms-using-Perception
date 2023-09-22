This is the official code base for the publication [Heuristic Optimization of Bat Algorithm for Heterogeneous Swarms using Perception](https://oresta.org/article-view/?id=572) published in Operational Research in Engineering Sciences:
Theory and Applications Journal

## Graphical Abstract
![Graphical Abstract_Swarm Paper](https://github.com/LavaHawk0123/LavaHawk0123/assets/75236655/555dad2a-9965-4d01-a811-243faedac81f)

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

Realsense Plug-In Source Repository: https://github.com/issaiass/realsense2_description

## Tools & Technologies Used:
**Software and Frameworks**
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![ROS Package Index](https://img.shields.io/ros/v/noetic/std_msgs)
![TensorFlow](https://img.shields.io/badge/TensorFlow%20-%23FF6F00.svg?&style=plastic&logo=TensorFlow&logoColor=white)
![Keras](https://img.shields.io/badge/Keras%20-%23D00000.svg?&style=plastic&logo=Keras&logoColor=white) 
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)
![Linux](https://img.shields.io/badge/-Linux-000000?style=flat&logo=linux&logoColor=FCC624)
![Ubuntu](https://img.shields.io/badge/-Ubuntu-E95420?style=plastic&logo=Ubuntu&logoColor=white)

**Languages**
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![C++](https://img.shields.io/badge/-C%2B%2B-00599C?style=plastic&logo=C%2B%2B)
![CMake](https://img.shields.io/badge/-CMake-064F8C?style=plastic&logo=CMake)
![C](https://img.shields.io/badge/-C-A8B9CC?style=plastic&logo=C)
![LaTeX](https://img.shields.io/badge/-LaTeX-008080?style=plastic&logo=LaTex)

**VCS**
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
![Git](https://img.shields.io/badge/-Git-black?style=plastic&logo=git)
![GitHub](https://img.shields.io/badge/-GitHub-181717?style=plastic&logo=github)
![GitLab](https://img.shields.io/badge/-GitLab-FCA121?style=plastic&logo=GitLab)

**Editors and IDEs**
&nbsp;
![VS Code](https://img.shields.io/badge/-VS%20Code-007ACC?style=plastic&logo=visual-studio-code)

