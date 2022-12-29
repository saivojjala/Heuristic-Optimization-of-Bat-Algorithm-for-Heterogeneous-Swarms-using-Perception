#! /usr/bin/env python3

# import ros stuff
import math
import rospy
import actionlib
import angles

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
# from bat_algo.srv import GoToPoint, GoToPointResponse  
from std_msgs.msg import Float64
from bat_algo.msg import GoToPointAction, GoToPointGoal, GoToPointResult, GoToPointFeedback

# robot state variables
active_ = True

position_ = Point()
yaw_ = 0
avg_desired_yaw_= 0
desired_yaw_ = 0
err_yaw_ = 0
# machine state
state_ = bool(False)
was_stuck_ = bool(False)
# goal
# desired_position_ = Point()
# desired_position_.x = 0
# desired_position_.y = 0
# desired_position_.z = 0
# global goal
global_position_ = Point()
global_position_.x = rospy.get_param('des_pos_x')
global_position_.y = rospy.get_param('des_pos_y')
global_position_.z = 0
# Action server
act_server_go_to_point_ = None
# publishers
pub_ = None
name_space_ = rospy.get_namespace()
# parameters
yaw_precision_ = 0.069 #0.01745329  #45  # +/- 4 degree allowed , 0.069 rad
dist_precision_ = 0.2
laser_val_ = 0.0
# global twist msg
twist_msg_ = Twist()

# def angular_error(desired_yaw_):
#     global yaw_
#     if desired_yaw_ < 0:
#         comp_desired_yaw_ = 2*math.pi + desired_yaw_
#     else:
#         comp_desired_yaw_ = desired_yaw_
#     if yaw_ < 0:
#         comp_yaw = 2*math.pi + yaw_
#     else:
#         comp_yaw = yaw_
#     error = abs(comp_desired_yaw_ - comp_yaw)
#     return error

# def angular_error(desired_yaw_):
#     global yaw_
#     if desired_yaw_ > 3.106686:
#         desired_yaw_ = 3.106686
#     elif desired_yaw_ < -3.106686:
#         desired_yaw_ = -3.106686
    
#     error = desired_yaw_ - yaw_
#     return error


def go_to_point(des_pos):
    global yaw_, pub_, yaw_precision_, state_, position_, twist_msg_, dist_precision_, err_yaw_, desired_yaw_, avg_desired_yaw_
    # desired_yaw_ = math.atan((des_pos.y-position_.y) / (des_pos.x-position_.x))

    # so you need to take the minimum of the absolute values of the 
    # differences of the angles instead, and then compensate for sign.
    # prev_err_yaw = err_yaw_
    # des_pos.x-position_.x 
    # prev_desired_yaw = avg_desired_yaw_
    _PI = math.pi
    desired_yaw_ = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    
    err_yaw_ = angles.shortest_angular_distance(yaw_, desired_yaw_)

    # err_yaw_ = desired_yaw_ - yaw_

    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos <= dist_precision_ and math.fabs(err_yaw_) <= yaw_precision_:
        twist_msg_.linear.x = 0
        twist_msg_.angular.z = 0
        pub_.publish(twist_msg_)
        state_ = True
        rospy.loginfo(msg = "[%s]\033[0;32m Reached Assigned Point\033[0m" %(name_space_))

    if err_pos <= dist_precision_:
        twist_msg_.linear.x = 0
        pub_.publish(twist_msg_)
        rospy.loginfo_throttle(period=1, msg = "[%s] Stopped Moving" %(name_space_))

    if math.fabs(err_yaw_) <= yaw_precision_:
        twist_msg_.angular.z = 0
        pub_.publish(twist_msg_)
        rospy.loginfo_throttle(period=1, msg = "[%s] Stopped Turning" %(name_space_))

    if err_pos > dist_precision_:
        err_pos = max(min(err_pos, 0.8), 0.03)
        speed_reduction_factor = 1.0-(abs(err_yaw_)/_PI)  # 1.0 -> 0.0
        twist_msg_.linear.x = (err_pos*err_pos) * speed_reduction_factor
        # twist_msg_.angular.z = 0
        pub_.publish(twist_msg_)
        rospy.loginfo_throttle(period=1.0, msg = "[%s] Go_straight: %.4f" %(name_space_, err_pos))

    if math.fabs(err_yaw_) > yaw_precision_:
        # if des_pos.y - position_.y > 0:
        # if err_yaw_ < 0:
        #     twist_msg_.angular.z = -(err_yaw_**2) - 0.1
        # else:
        #     twist_msg_.angular.z =  (err_yaw_**2) + 0.1
        # else:
        #     if err_yaw_ > 0:
        #         twist_msg_.angular.z = -(err_yaw_**2) - 0.1
        #     else:
        #         twist_msg_.angular.z =  (err_yaw_**2) + 0.1
        sign = err_yaw_/math.fabs(err_yaw_)
        twist_msg_.angular.z = sign*max(min(abs(err_yaw_), 0.8), 0.1)  # **3
        # twist_msg_.linear.x = 0
        pub_.publish(twist_msg_)
        rospy.loginfo_throttle(period=0.04, msg = "[%s] Desired: %.4f Actual: %.4f" %(name_space_, desired_yaw_, yaw_))


# action callback
def go_to_point_clbk(goal):
    global state_, active_, act_server_go_to_point_, global_position_, position_, was_stuck_
    # feedbk = GoToPointFeedback()
    # calc dist btw both points
    # active_ = True
    state_ = False
    # twist_msg = Twist()
    # twist_msg.linear.x = 0
    # twist_msg.angular.z = 0
    rospy.loginfo("[%s] \033[0;36mGot request, executing callback\033[0m" %name_space_)
    rate = rospy.Rate(100)
    
    counter = 0
    if laser_val_ <= 0.5:
        rospy.loginfo("[%s]\033[0;31m Currently Stuck\033[0m" %name_space_)
        was_stuck_ = True
    
    while True:
        
        err_global_pos = math.sqrt(math.pow(global_position_.x - position_.x, 2) + math.pow(global_position_.y - position_.y, 2))
        
        # ? Condition for when robot encounters obstacle first time
        if laser_val_<=0.5 and counter>560 and not was_stuck_ :  # approx 5 sec of trying
            # ? Stop moving and Exit
            rospy.loginfo("[%s]\033[0;31m Still Stuck after 5 secs of trying\033[0m" %name_space_)
            twist_msg_.linear.x = 0
            twist_msg_.angular.z = 0
            pub_.publish(twist_msg_)
            state_ = True
        
        # ? Condition for when robot is free from obstacle
        elif laser_val_> 0.7 and was_stuck_:
            rospy.loginfo("[%s]\033[0;31m No Longer Stuck\033[0m" %name_space_)
            was_stuck_ = False
            twist_msg_.angular.z = 0
            pub_.publish(twist_msg_)
        
        # ? Close to Global Goal
        elif err_global_pos < 0.5:
            twist_msg_.linear.x = 0
            twist_msg_.angular.z = 0
            pub_.publish(twist_msg_)
            rospy.loginfo(msg = "[%s] \033[0;33mReached Global Goal on the way\033[0m" %(name_space_))
            state_ = True
        
        # ? Recovery Condition
        elif laser_val_<=0.5 and was_stuck_:
            # ? Make robot rotate till laser_val is > 0.5
            twist_msg_.angular.z = 0.2 
            pub_.publish(twist_msg_)
            # ? Rotated for some time and no way out
            if counter>3142:#Time taken to complete full rotation
                twist_msg_.angular.z = 0
                pub_.publish(twist_msg_)
                state_ = True
                rospy.loginfo("[%s]\033[0;31m Stuck, no way out\033[0m" %name_space_)
            else:
                rospy.loginfo_throttle(period=2,msg="[%s]\033[0;31m Executing recovery behavior\033[0m" %name_space_)
        
        else:
            go_to_point(goal)
        
        # pub_.publish(twist_msg_)
        
        if state_ == True:
            break
        
        counter+=1
        
        rate.sleep()

    result = GoToPointResult()
    pos_x, pos_y = position_.x, position_.y
    result.distance = math.sqrt(math.pow(global_position_.x - pos_x, 2) + math.pow(global_position_.y - pos_y, 2))
    result.x, result.y = pos_x, pos_y 
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

    pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1 )
    sub_odom = rospy.Subscriber('odom', Odometry, odom_clbk)
    sub_laser = rospy.Subscriber('min_laser', Float64, min_laser_clbk)

    act_server_go_to_point_ = actionlib.SimpleActionServer('go_to_point', GoToPointAction, go_to_point_clbk, False)
    act_server_go_to_point_.start()


if __name__ == '__main__':
    main()
