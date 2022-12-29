#! /usr/bin/env python3

import rospy
import math 

from tf import transformations
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

act_server_go_to_point_ = None
rate = None
vel = Twist()
position = Point()
goal_position = Point()
goal_position.x = -1
goal_position.y = 5
name_space_ = rospy.get_namespace()
yaw = 0
desired_yaw = 0
error_yaw = 0
error_distance = 0
laser_scan = None
laser_val = math.inf
pub = None
PI = math.pi
yaw_precision = 4
dist_precision = 0.25

def rotation():      
    if(abs(error_yaw)>yaw_precision):
        vel.linear.x = error_distance*0.12
        vel.angular.z = error_yaw*0.01
        pub.publish(vel)
    else:
        vel.linear.x=0
        vel.angular.z=0
        pub.publish(vel)
        go_straight()

def go_straight():
    vel.linear.x = error_distance*0.12
    vel.angular.z=0
    pub.publish(vel)

def obstacle_avoidance():     
        vel.linear.x = 0
        vel.linear.z = 1.0
        pub.publish(vel)
    
def decision():
    if laser_val>1:
        rospy.loginfo("Global Planner")
        if error_distance>dist_precision:
            rotation()
        else:
            vel.linear.x=0
            vel.angular.z=0
            pub.publish(vel)
            rospy.signal_shutdown("Traversal Complete")
    elif laser_val<1:   
        rospy.loginfo("Obstacle Avoidance")
        obstacle_avoidance()        
    else:
        rospy.loginfo("Unknown Case")
    
def laser_clbk(msg):
    global laser_scan,laser_val, desired_yaw, yaw, error_yaw, error_distance, yaw_precision, dist_precision
    laser_scan = msg.ranges
    laser_val = min(min(msg.ranges[0:5]), 1.5)
    error_distance = math.sqrt(math.pow(goal_position.x - position.x, 2) + math.pow(goal_position.y - position.y, 2))
    desired_yaw = math.atan2(goal_position.y - position.y, goal_position.x - position.x)
    error_yaw = (desired_yaw - yaw)*180/PI
    decision()

def odom_clbk(msg):
    global position, yaw
    position = msg.pose.pose.position
    quaternion = (msg.pose.pose.orientation.x, 
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    
def main():
    global pub, act_server_go_to_point_, rate
    pub = rospy.Publisher('/bat_swarm/bat_1/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(100)
    rospy.Subscriber('/bat_swarm/bat_1/odom', Odometry, odom_clbk)
    rospy.Subscriber('/bat_swarm/bat_1/LaserScan', LaserScan, laser_clbk)
    rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('globalPlanner')
    main()