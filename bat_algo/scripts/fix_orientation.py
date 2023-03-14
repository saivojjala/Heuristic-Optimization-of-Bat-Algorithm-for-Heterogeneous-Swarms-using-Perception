#! /usr/bin/env python3

import rospy
import math
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion

PI_ = math.pi
rate_ = None
pub_ = None
vel_ = Twist()
name_space_ = rospy.get_namespace()
goal_orientation_ = Quaternion()
goal_orientation_.x = rospy.get_param('des_ori_x')
goal_orientation_.y = rospy.get_param('des_ori_y')
goal_orientation_.z = rospy.get_param('des_ori_z')
goal_orientation_.w = rospy.get_param('des_ori_w')
fix_yaw_ = rospy.get_param('fix_orientation')
error_yaw_ = 0
yaw_precision_ = 4

def rotation():
    if(abs(error_yaw_)>yaw_precision_):
        vel_.linear.x = 0
        vel_.angular.z = 0.5
        pub_.publish(vel_)
    else:
        vel_.linear.x = 0
        vel_.angular.z = 0
        pub_.publish(vel_)
        rospy.loginfo(msg = "[%s]\033[0;32m Orientation Corrected\033[0m" %(name_space_))
        rospy.signal_shutdown("Traversal Complete")

def odom_clbk(msg):
    global error_yaw_, PI_
    quaternion = (msg.pose.pose.orientation.x, 
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    euler = transformations.euler_from_quaternion(goal_orientation_)
    final_yaw_ = euler[2]
    error_yaw_ = (final_yaw_ - yaw_)*180/PI_
    if(fix_yaw_==True):
        rotation()
        
def main():
    global pub_, act_server_go_to_point_, rate_
    pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('odom', Odometry, odom_clbk)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('fixOrientation')
    main()