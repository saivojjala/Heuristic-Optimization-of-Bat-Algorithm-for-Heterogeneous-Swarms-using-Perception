#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry 

def goal1_clbk(msg):
    rospy.set_param('/bat_swarm_red/bat_1/des_pos_x', msg.pose.pose.position.x-3)
    rospy.set_param('/bat_swarm_red/bat_1/des_pos_y', msg.pose.pose.position.y+3)
    rospy.set_param('/bat_swarm_red/bat_2/des_pos_x', msg.pose.pose.position.x-1.5)
    rospy.set_param('/bat_swarm_red/bat_2/des_pos_y', msg.pose.pose.position.y+1.5)
    rospy.set_param('/bat_swarm_red/bat_3/des_pos_x', msg.pose.pose.position.x)
    rospy.set_param('/bat_swarm_red/bat_3/des_pos_y', msg.pose.pose.position.y)
    rospy.set_param('/bat_swarm_red/bat_4/des_pos_x', msg.pose.pose.position.x-1.5)
    rospy.set_param('/bat_swarm_red/bat_4/des_pos_y', msg.pose.pose.position.y-1.5)
    rospy.set_param('/bat_swarm_red/bat_5/des_pos_x', msg.pose.pose.position.x-3)
    rospy.set_param('/bat_swarm_red/bat_5/des_pos_y', msg.pose.pose.position.y-3)


def goal2_clbk(msg):
    rospy.set_param('/bat_swarm_green/bat_6/des_pos_x', msg.pose.pose.position.x-3)
    rospy.set_param('/bat_swarm_green/bat_6/des_pos_y', msg.pose.pose.position.y+3)
    rospy.set_param('/bat_swarm_green/bat_7/des_pos_x', msg.pose.pose.position.x-1.5)
    rospy.set_param('/bat_swarm_green/bat_7/des_pos_y', msg.pose.pose.position.y+1.5)
    rospy.set_param('/bat_swarm_green/bat_8/des_pos_x', msg.pose.pose.position.x)
    rospy.set_param('/bat_swarm_green/bat_8/des_pos_y', msg.pose.pose.position.y)
    rospy.set_param('/bat_swarm_green/bat_9/des_pos_x', msg.pose.pose.position.x-1.5)
    rospy.set_param('/bat_swarm_green/bat_9/des_pos_y', msg.pose.pose.position.y-1.5)
    rospy.set_param('/bat_swarm_green/bat_10/des_pos_x', msg.pose.pose.position.x-3)
    rospy.set_param('/bat_swarm_green/bat_10/des_pos_y', msg.pose.pose.position.y-3)


def goal3_clbk(msg):
    rospy.set_param('/bat_swarm_blue/bat_11/des_pos_x', msg.pose.pose.position.x-3)
    rospy.set_param('/bat_swarm_blue/bat_11/des_pos_y', msg.pose.pose.position.y+3)
    rospy.set_param('/bat_swarm_blue/bat_12/des_pos_x', msg.pose.pose.position.x-1.5)
    rospy.set_param('/bat_swarm_blue/bat_12/des_pos_y', msg.pose.pose.position.y+1.5)
    rospy.set_param('/bat_swarm_blue/bat_13/des_pos_x', msg.pose.pose.position.x)
    rospy.set_param('/bat_swarm_blue/bat_13/des_pos_y', msg.pose.pose.position.y)
    rospy.set_param('/bat_swarm_blue/bat_14/des_pos_x', msg.pose.pose.position.x-1.5)
    rospy.set_param('/bat_swarm_blue/bat_14/des_pos_y', msg.pose.pose.position.y-1.5)
    rospy.set_param('/bat_swarm_blue/bat_15/des_pos_x', msg.pose.pose.position.x-3)
    rospy.set_param('/bat_swarm_blue/bat_15/des_pos_y', msg.pose.pose.position.y-3)

def main():
    rospy.Subscriber('/global_pose_1', Odometry, goal1_clbk)
    rospy.Subscriber('/global_pose_2', Odometry, goal2_clbk)
    rospy.Subscriber('/global_pose_3', Odometry, goal3_clbk)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('setGoalParams')
    main()