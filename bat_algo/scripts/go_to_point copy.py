#! /usr/bin/env python3

# import ros stuff]
import rospy
import actionlib

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
# from bat_algo.srv import GoToPoint, GoToPointResponse  
from std_msgs.msg import Float64
from bat_algo.msg import GoToPointAction, GoToPointGoal, GoToPointResult, GoToPointFeedback

import math

# robot state variables
active_ = True

position_ = Point()
yaw_ = 2
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0
# global goal
global_position_ = Point()
global_position_.x = rospy.get_param('/bat_swarm/des_pos_x')
global_position_.y = rospy.get_param('/bat_swarm/des_pos_y')
global_position_.z = 0
# Action server
act_server_go_to_point_ = None
# publishers
pub_ = None
name_space_ = rospy.get_namespace()
# parameters
yaw_precision_ = math.pi / 45 #45  # +/- 4 degree allowed
dist_precision_ = 0.02
laser_val_ = 0.0
prev_clkwse_ = False

twist_msg_ = Twist()
    
def change_state(state):
    global state_
    state_ = state
    # rospy.loginfo('[%s] State changed to [%s]' %(name_space_, state_))


def fix_yaw(des_pos):
    global yaw_, pub_, yaw_precision_, state_, position_, prev_clkwse_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = math.fabs(desired_yaw) - math.fabs(yaw_)
    # yaw_val = math.fabs(err_yaw)/math.fabs(desired_yaw)

    if math.fabs(err_yaw) <= yaw_precision_:
        twist_msg_.angular.z = 0
        pub_.publish(twist_msg_)
        change_state(1)
        # rospy.loginfo('Yaw error: [%s]' % err_yaw)
    else: #if math.fabs(err_yaw) > yaw_precision_:
        if laser_val_ < 0.7:
            if prev_clkwse_ is True:
                twist_msg_.angular.z =  math.fabs(err_yaw)
                prev_clkwse_ = False
            else:
                twist_msg_.angular.z = -math.fabs(err_yaw)
                prev_clkwse_ = True
        else:
            twist_msg_.angular.z = -err_yaw #if err_yaw > 0 else err_yaw  # >
        
        pub_.publish(twist_msg_)

    # state change conditions


def go_straight(des_pos):
    global yaw_, pub_, yaw_precision_, state_, position_, laser_val_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    inv_dist_to_obstacle = laser_val_ * 0.5 # random trial and error

    if math.fabs(err_yaw) > yaw_precision_:
        # print('Yaw error: [%s]' % err_yaw)
        # twist_msg_.linear.x = 0.00
        # pub_.publish(twist_msg_)
        change_state(0)

    elif err_pos > dist_precision_:
        twist_msg_.linear.x = err_pos * inv_dist_to_obstacle * 0.2 
    else:
        # twist_msg_.linear.x = 0
        # pub_.publish(twist_msg_)
        change_state(2)
    pub_.publish(twist_msg_)


def go_to_goal(des_pos):
    global position_, yaw_, pub_, state_
    # X = position_.x
    # Y = position_.y
    # x_goal = des_pos.x
    # y_goal = des_pos.y
    # while(True):
    K_linear=0.2
    distance=abs(math.sqrt(((des_pos.x-position_.x)**2)+((des_pos.y-position_.y)**2)))
    linear_speed=distance*K_linear
    K_angular=1.0
    desired_angle_goal=math.atan2(des_pos.y-position_.y, des_pos.x-position_.x)
    angular_speed=(desired_angle_goal-yaw_)*K_angular
    twist_msg_.linear.x=min(linear_speed,0.4)
    twist_msg_.angular.z=min(angular_speed, 0.8)
    pub_.publish(twist_msg_)
    # print('x=',X,'y=',Y)
    # if laser_val_ < 0.45:
    #     twist_msg_.linear.x = 0.00
    #     rospy.logwarn( 'Too close to obstace: [%s]' % laser_val_)
    #     change_state(2)
    if (distance<dist_precision_):
        print( '[%s] Position error: [%s]' %(rospy.get_namespace(), distance))
        change_state(2)
        # break


def done():
    global pub_, active_
    active_ = False
    twist_msg_ = Twist()
    twist_msg_.linear.x = 0
    twist_msg_.angular.z = 0
    pub_.publish(twist_msg_)


# action callback
def go_to_point_clbk(goal):
    global state_, active_, act_server_go_to_point_, desired_position_, global_position_, position_
    # feedbk = GoToPointFeedback()
    # calc dist btw both points
    active_ = True
    state_ = 0
    # twist_msg_ = Twist()
    # twist_msg_.linear.x = 0
    # twist_msg_.angular.z = 0
    rospy.loginfo("[%s] Got request, executing callback" %name_space_)
    rate = rospy.Rate(20)
    while active_ == True:
        if state_ == 0:
            rospy.loginfo_throttle(period=1, msg = "[%s] Fix_yaw" % name_space_)
            fix_yaw(goal)
        elif state_ == 1:
            rospy.loginfo_throttle(period=1, msg = "[%s] Go_straight" % name_space_)
            # go_straight(desired_position_)
            go_to_goal(goal)
        elif state_ == 2:
            rospy.loginfo(msg = "[%s] Done" % name_space_) #  _throttle(period=0.5, 
            done()
        rate.sleep()

    result = GoToPointResult()
    result.distance = math.sqrt(math.pow(global_position_.x - position_.x, 2) + math.pow(global_position_.y - position_.y, 2))
    act_server_go_to_point_.set_succeeded(result, "[%s] Reached Point Successfully" % name_space_)
    return

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

# min_laser calback
def min_laser_clbk(val):
    global laser_val_
    laser_val_ = val.data

def main():
    global pub_, act_server_go_to_point_
    rospy.init_node('go_to_point_action_server')

    pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('odom', Odometry, odom_clbk)
    sub_laser = rospy.Subscriber('min_laser', Float64, min_laser_clbk)
    
    act_server_go_to_point_ = actionlib.SimpleActionServer('go_to_point', GoToPointAction, go_to_point_clbk, False)
    act_server_go_to_point_.start()

if __name__ == '__main__':
    main()
