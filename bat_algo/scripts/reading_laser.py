#! /usr/bin/env python3

import math
import rospy

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

from sensor_msgs.msg import LaserScan
from std_msgs.msg import *
from geometry_msgs.msg import Twist

min_laser_pub_ = None
min_val_ = 5.0

def clbk_laser(msg):
    global min_laser_pub_, min_val_
    min_val_ = min(min(msg.ranges[0:5]), 1.35) - 0.35
    min_laser_pub_.publish(min_val_)


def main():
    global min_laser_pub_, count_, old_count_
    rospy.init_node('reading_laser')

    # pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    # twist = Twist()
    # twist.linear.x = 0
    # twist.linear.y = 0
    # twist.angular.z = 0
    # pub.publish(twist)
    # pub_.unregister()

    min_laser_pub_ = rospy.Publisher("min_laser", Float64, queue_size=1)
    # rot_pub = rospy.Publisher("laser_controller/command", Float64, queue_size=5)  # 1

    # srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # # set robot position
    # model_state = ModelState()
    # model_state.model_name = rospy.get_param('robot_name')
    # model_state.pose.position.x = rospy.get_param('initial_x')
    # model_state.pose.position.y = rospy.get_param('initial_y')
    # resp = srv_client_set_model_state(model_state)

    rospy.Subscriber('LaserScan', LaserScan, clbk_laser)
    rospy.spin()
    # rot_msg = Float64()
    # rot_msg.data = 0.00

    # rate = rospy.Rate(60)
    # while not rospy.is_shutdown():
    #     rot_pub.publish(rot_msg)
    #     rot_msg.data += 0.01*math.pi
    #     if rot_msg.data >= math.pi*2:
    #         rot_msg.data = 0.00
    #         count_ += 1     # every rotation update
    #     rate.sleep()

if __name__ == '__main__':
    main()
