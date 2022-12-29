#! /usr/bin/env python3

import rospy
import math 
import actionlib

from tf import transformations
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_srvs.srv import *
from bat_algo.msg import GoToPointAction, GoToPointResult

act_server_go_to_point_ = None
rate_ = None
vel_ = Twist()
position_ = Point()
goal_position_ = Point()
goal_position_.x = rospy.get_param('des_pos_x')
goal_position_.y = rospy.get_param('des_pos_y')
name_space_ = rospy.get_namespace()
result_ = GoToPointResult()
yaw_ = 0
desired_yaw_ = 0
error_yaw_ = 0
error_distance_ = 0
laser_val_ = math.inf
pub_ = None
PI_ = math.pi
yaw_precision_ = 4
dist_precision_ = 0.25

def rotation():      
    if(abs(error_yaw_)>yaw_precision_):
        # vel_.linear.x = error_distance_*0.12
        vel_.angular.z = error_yaw_*0.01
        pub_.publish(vel_)
    else:
        vel_.linear.x=0
        vel_.angular.z=0
        pub_.publish(vel_)
        go_straight()

def go_straight():
    vel_.linear.x = error_distance_*0.12
    vel_.angular.z=0
    pub_.publish(vel_)

def obstacle_avoidance():     
        vel_.linear.x = 0
        vel_.linear.z = 1.0
        pub_.publish(vel_)
    
def decision(goal):
    global act_server_go_to_point_
    rate_ = rospy.Rate(100)
    # rospy.loginfo(goal)
    if laser_val_>1:
        rospy.loginfo("Global Planner")
        if error_distance_>dist_precision_:
            rotation()
        else:
            vel_.linear.x=0
            vel_.angular.z=0
            pub_.publish(vel_)
            rospy.signal_shutdown("Traversal Complete")
    elif laser_val_<=1:   
        rospy.loginfo("Obstacle Avoidance")
        obstacle_avoidance()
    else:
        rospy.loginfo("Unknown Case")

    rate_.sleep()
    result_.x, result_.y = position_.x, position_.y
    result_.distance = error_distance_
    act_server_go_to_point_.set_succeeded(result_,  "[%s] Reached Point Successfully" % name_space_)
    
def laser_clbk(msg):
    global laser_val_, error_distance_, error_yaw_
    laser_val_ = min(min(msg.ranges[0:5]), 1.5)
    error_distance_ = math.sqrt(math.pow(goal_position_.x - position_.x, 2) + math.pow(goal_position_.y - position_.y, 2))
    desired_yaw_ = math.atan2(goal_position_.y - position_.y, goal_position_.x - position_.x)
    error_yaw_ = (desired_yaw_ - yaw_)*180/PI_
    # decision()

def odom_clbk(msg):
    global position_, yaw_
    position_ = msg.pose.pose.position
    quaternion = (msg.pose.pose.orientation.x, 
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    
def main():
    global pub_, act_server_go_to_point_, rate_
    pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('odom', Odometry, odom_clbk)
    rospy.Subscriber('LaserScan', LaserScan, laser_clbk)
    act_server_go_to_point_ = actionlib.SimpleActionServer('go_to_point', GoToPointAction, decision, False)
    act_server_go_to_point_.start()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('globalPlanner')
    main()