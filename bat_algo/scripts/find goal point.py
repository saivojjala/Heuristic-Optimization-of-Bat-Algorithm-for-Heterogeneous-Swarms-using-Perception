#! /usr/bin/env python3

# import ros stuff
import actionlib
import rospy

# import ros message
from geometry_msgs.msg import Point, Twist
from scripts.go_to_point_combined import go_to_point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import * # Float32, Float64

# from tf import transformations
from tf import transformations

# import ros service
from std_srvs.srv import *
from bat_algo.msg import Float32List
from bat_algo.msg import GoToPointAction, GoToPointGoal, GoToPointFeedback, GoToPointResult

import math

import random 
import numpy as np

# location variables
Nxt_Pos_ = [0.0, 0.0]
Cur_Pos_ = [0.0, 0.0]
max_ = 1.0
min_ = 0.3
# publishers
act_client_go_to_point_ = None
cmd_vel_publisher_ = None
# robot variables
laser_val_ = 10.0
name_space_ = rospy.get_namespace()

def move_random():
    global Nxt_Pos_, min_, max_
    for n in range(2):
        rnd = np.random.uniform(0, 1)
        Nxt_Pos_[n] = min_+(max_-min_)*rnd
    result = go_to_point(Nxt_Pos_)
    
def go_to_point(Pos):
    global act_client_go_to_point_, Cur_Pos_, name_space_
    if Pos[0] != Cur_Pos_[0] and Pos[1] != Cur_Pos_[1]:
        rospy.loginfo('[%s] Moving robot to Nxt_Pos:[ %.2f , %.2f ] from [ %.2f , %.2f ] ' \
                     % (name_space_, Pos[0], Pos[1], Cur_Pos_[0], Cur_Pos_[1]))
    goal = GoToPointGoal()
    goal.x = Pos[0]
    goal.y = Pos[1]
    act_client_go_to_point_.send_goal(goal)#, feedback_cb=feedback_cb)
    act_client_go_to_point_.wait_for_result()
    # rospy.loginfo('[%s] Robot reached Nxt_Pos' %(name_space_))
    result = act_client_go_to_point_.get_result()
    if goal.x != result.x and goal.y != result.y:
        rospy.loginfo("[%s] Could not reach, currently at (%.2f, %.2f)" %(name_space_ ,result.x, result.y))
    return result


def min_laser_clbk(val):
    global laser_val_
    laser_val_ = val.data

# Odometry Callback
def odom_clbk(msg):
    global position_
    global yaw_
    # position
    position_ = msg.pose.pose.position
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]



def main():
    global fit_pos_publisher_, gen_publisher_, act_client_go_to_point_
        
    rospy.loginfo("Starting_node: move_bat")
    rospy.init_node("move_bat")

        
    act_client_go_to_point_ = actionlib.SimpleActionClient( 'go_to_point', GoToPointAction )
    
    sub_laser = rospy.Subscriber('min_laser', Float64, min_laser_clbk)
    # sub_best_bat = rospy.Subscriber('/bat_swarm/best_bat_data', Float32List, best_bat_clbk)
    # sub_gen_check = rospy.Subscriber('/bat_swarm/common_gen', Float32List, gen_checker_clbk)
    sub_odom = rospy.Subscriber('odom', Odometry, odom_clbk)

    
    rospy.loginfo("[%s] \033[0;32mReached Goal !!!\033[0m" %(name_space_))
    
    sub_laser.unregister()
    
    bat_index = int(name_space_[-2])-1
    rospy.spin()

if __name__ == "__main__":
    main()