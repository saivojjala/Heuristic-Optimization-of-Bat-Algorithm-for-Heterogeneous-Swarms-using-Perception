#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, Quaternion

goal1_pose_ = Odometry()
goal2_pose_ = Odometry()
goal3_pose_ = Odometry()

def main():
    rate = rospy.Rate(10)
    pub1 = rospy.Publisher('/global_pose_1', Odometry, queue_size=1)
    pub2 = rospy.Publisher('/global_pose_2', Odometry, queue_size=1)
    pub3 = rospy.Publisher('/global_pose_3', Odometry, queue_size=1)

    while True:
        goal1_pose_.pose.pose.position.x = -5
        goal1_pose_.pose.pose.position.y = 0
        goal1_pose_.pose.pose.orientation.x = -0.001299681337891499
        goal1_pose_.pose.pose.orientation.y = -3.904208232131656e-06
        goal1_pose_.pose.pose.orientation.z = -2.5455883476238017e-06
        goal1_pose_.pose.pose.orientation.w = 0.999999155402992
        
        goal2_pose_.pose.pose.position.x = -5
        goal2_pose_.pose.pose.position.y = -6
        goal2_pose_.pose.pose.orientation.x = -0.001299681337891499
        goal2_pose_.pose.pose.orientation.y = -3.904208232131656e-06
        goal2_pose_.pose.pose.orientation.z = -2.5455883476238017e-06
        goal2_pose_.pose.pose.orientation.w = 0.999999155402992
        
        goal3_pose_.pose.pose.position.x = -5
        goal3_pose_.pose.pose.position.y = 6
        goal3_pose_.pose.pose.orientation.x = -0.001299681337891499
        goal3_pose_.pose.pose.orientation.y = -3.904208232131656e-06
        goal3_pose_.pose.pose.orientation.z = -2.5455883476238017e-06
        goal3_pose_.pose.pose.orientation.w = 0.999999155402992

        pub1.publish(goal1_pose_)
        pub2.publish(goal2_pose_)
        pub3.publish(goal3_pose_)

        rate.sleep()
        
if __name__ == "__main__":
    rospy.init_node('pubGoalParams')
    main()