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

# correct_yaw_=bool(False)
# state_=bool(False)
act_server_go_to_point_ = None
rate_ = None
vel_ = Twist()
position_ = Point()
goal_position_ = Point()
goal_position_.x = rospy.get_param('des_pos_x')
goal_position_.y = rospy.get_param('des_pos_y')
# goal_position_.x = -1
# goal_position_.y = -5
name_space_ = rospy.get_namespace()
result_ = GoToPointResult()
yaw_ = 0
desired_yaw_ = 0
error_yaw_ = 0
error_distance_ = 0
# laser_val_ = math.inf
regions = {}
pub_ = None
PI_ = math.pi
yaw_precision_ = 4
dist_precision_ = 0.25

def rotation():      
    while regions['front']<=0.7 or regions['right']<=0.7 or regions['left']<=0.7:   
        rospy.loginfo("[%s]\033[0;31m Currently Stuck, Executing Obstacle Avoidance\033[0m" %name_space_)
        obstacle_avoidance()

    if(abs(error_yaw_)>yaw_precision_):
        vel_.linear.x = error_distance_*0.12
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
    global regions
    print(regions)
    if regions['front']<=0.7 and regions['right']<=0.7 and regions['left']>0.7:
        vel_.linear.x = regions['right']*0.5
        vel_.angular.z = 1.0
        pub_.publish(vel_)
    elif regions['front']<=0.7 and regions['right']>0.7 and regions['left']<=0.7:
        vel_.linear.x = regions['left']*0.3
        vel_.angular.z = -1.0
        pub_.publish(vel_)
    elif regions['front']<=0.7 and regions['right']>0.7 and regions['left']>0.7:
        vel_.linear.x = regions['left']*0.3
        vel_.angular.z = -1.0
        pub_.publish(vel_)
    elif regions['front']<=0.7 and regions['right']<=0.7 and regions['left']<=0.7:
        vel_.linear.x = -0.2
        vel_.angular.z = -1.0
        pub_.publish(vel_)

def decision(goal):
    global act_server_go_to_point_ #state_, correct_yaw_
    
    rate_ = rospy.Rate(100)
    rospy.loginfo("[%s] \033[0;36mGot request, executing callback\033[0m" %name_space_)
    # rospy.loginfo(goal)
    
    while True:
        if error_distance_<dist_precision_:
            vel_.linear.x=0
            vel_.angular.z=0
            pub_.publish(vel_)
            rospy.loginfo(msg = "[%s]\033[0;32m Reached Assigned Point\033[0m" %(name_space_))
            break
            # rospy.signal_shutdown("Traversal Complete")
            # state_=True
        elif error_distance_>dist_precision_:
            rospy.loginfo("[%s] \033[0;33mExecuting Global Planner\033[0m" %name_space_)
            rotation()
        else:
            rospy.loginfo("[%s]\033[0;31m Unknown Case\033[0m" %name_space_)

        # final_yaw = 3.01
        # error = (final_yaw - yaw_)*180/PI_
        # if (abs(error) > yaw_precision_) and state_:
        #     vel_.linear.x=0
        #     vel_.angular.z = error*0.01
        #     pub_.publish(vel_)
        # else:
        #     correct_yaw_ = True

        # if state_ and correct_yaw_:
        #     break

        rate_.sleep()

    result_.x, result_.y = position_.x, position_.y
    result_.distance = math.sqrt(math.pow(goal_position_.x - position_.x, 2) + math.pow(goal_position_.y - position_.y, 2))
    act_server_go_to_point_.set_succeeded(result_,  "[%s] Reached Point Successfully" % name_space_)
    return

def laser_clbk(msg):
    global laser_val_, error_distance_, error_yaw_, regions
    regions = { 'right':  min(min(msg.ranges[0:5]), 1), 'front':  min(min(msg.ranges[6:10]), 1), 'left':   min(min(msg.ranges[11:15]), 1) }
    # laser_val_ = min(min(msg.ranges[0:5]), 1.5)
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