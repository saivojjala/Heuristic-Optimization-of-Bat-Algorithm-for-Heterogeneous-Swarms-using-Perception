#! /usr/bin/env python3

import rospy
import math
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathPublisher:
    def __init__(self) -> None:
        self.name_space_ = rospy.get_namespace()
        self.ros_path = Path()
        self.ros_path.header.frame_id = "map"
        self.path_pub = rospy.Publisher("path",Path,queue_size=1)
        pub_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.odom_clbk)


    def odom_clbk(self,msg):
        PosePoint = PoseStamped()
        (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position,msg.pose.pose.position)
        self.ros_path.poses.append(PosePoint)
        self.publishPathMsg()

    def publishPathMsg(self):
        self.path_pub.publish(self.path.ros_path)


if __name__ == '__main__':
    rospy.init_node('PublishPathNode')
    obj = PathPublisher()
    obj.main()
    rospy.spin()