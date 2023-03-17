#! /usr/bin/env python3

import rospy
import math
from tf import transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

ros_path = [Path(),Path(),Path(),Path(),Path(),Path(),Path(),Path(),Path(),Path(),Path(),Path(),Path(),Path(),Path()]
for i in range(0,15):
    ros_path[i].header.frame_id = "odom"
path_pub = None
bat_num = 0
pub1 = rospy.Publisher("/bat_swarm_red/bat_1/Path",Path,queue_size=1)
pub2 = rospy.Publisher("/bat_swarm_red/bat_2/Path",Path,queue_size=1)
pub3 = rospy.Publisher("/bat_swarm_red/bat_3/Path",Path,queue_size=1)
pub4 = rospy.Publisher("/bat_swarm_red/bat_4/Path",Path,queue_size=1)
pub5 = rospy.Publisher("/bat_swarm_red/bat_5/Path",Path,queue_size=1)
pub6 = rospy.Publisher("/bat_swarm_green/bat_6/Path",Path,queue_size=1)
pub7 = rospy.Publisher("/bat_swarm_green/bat_7/Path",Path,queue_size=1)
pub8 = rospy.Publisher("/bat_swarm_green/bat_8/Path",Path,queue_size=1)
pub9 = rospy.Publisher("/bat_swarm_green/bat_9/Path",Path,queue_size=1)
pub10 = rospy.Publisher("/bat_swarm_green/bat_10/Path",Path,queue_size=1)
pub11 = rospy.Publisher("/bat_swarm_blue/bat_11/Path",Path,queue_size=1)
pub12 = rospy.Publisher("/bat_swarm_blue/bat_12/Path",Path,queue_size=1)
pub13 = rospy.Publisher("/bat_swarm_blue/bat_13/Path",Path,queue_size=1)
pub14 = rospy.Publisher("/bat_swarm_blue/bat_14/Path",Path,queue_size=1)
pub15 = rospy.Publisher("/bat_swarm_blue/bat_15/Path",Path,queue_size=1)

publishers = [pub1,pub2,pub3,pub4,pub5,pub6,pub7,pub8,pub9,pub10,pub11,pub12,pub13,pub14,pub15]

def getTopicPath(index):
    red = [1,2,3,4,5]
    green= [6,7,8,9,10]
    blue= [11,12,13,14,15]
    if(index in red):
        return "/bat_swarm_red/bat_"+str(index)+"/odom"
    if(index in green):
        return "/bat_swarm_green/bat_"+str(index)+"/odom"
    if(index in blue):
        return "/bat_swarm_blue/bat_"+str(index)+"/odom"
    
def getTopicOdom(index):
    red = [1,2,3,4,5]
    green= [6,7,8,9,10]
    blue= [11,12,13,14,15]
    if(index in red):
        return "/bat_swarm_red/bat_"+str(index)+"/odom"
    if(index in green):
        return "/bat_swarm_green/bat_"+str(index)+"/odom"
    if(index in blue):
        return "/bat_swarm_blue/bat_"+str(index)+"/odom"

def odom_clbk1(msg):
    global ros_path,bat_num
    #print("Got Odom 1")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[0].poses.append(PosePoint)
    publishers[0].publish(ros_path[0])

def odom_clbk2(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[1].poses.append(PosePoint)
    publishers[1].publish(ros_path[1])

def odom_clbk3(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[2].poses.append(PosePoint)
    publishers[2].publish(ros_path[2])
    
def odom_clbk4(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[3].poses.append(PosePoint)
    publishers[3].publish(ros_path[3])
def odom_clbk5(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[4].poses.append(PosePoint)
    publishers[4].publish(ros_path[4])
def odom_clbk6(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[5].poses.append(PosePoint)
    publishers[5].publish(ros_path[5])
def odom_clbk7(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[6].poses.append(PosePoint)
    publishers[6].publish(ros_path[6])
def odom_clbk8(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[7].poses.append(PosePoint)
    publishers[7].publish(ros_path[7])
def odom_clbk9(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[8].poses.append(PosePoint)
    publishers[8].publish(ros_path[8])
def odom_clbk10(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[9].poses.append(PosePoint)
    publishers[9].publish(ros_path[9])
def odom_clbk11(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[10].poses.append(PosePoint)
    publishers[10].publish(ros_path[10])
def odom_clbk12(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[11].poses.append(PosePoint)
    publishers[11].publish(ros_path[11])
def odom_clbk13(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[12].poses.append(PosePoint)
    publishers[12].publish(ros_path[12])
def odom_clbk14(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[13].poses.append(PosePoint)
    publishers[13].publish(ros_path[13])
def odom_clbk15(msg):
    global ros_path,bat_num
    #print("Got Odom 2")
    PosePoint = PoseStamped()
    (PosePoint.pose.position.x,PosePoint.pose.position.y)= (msg.pose.pose.position.x,msg.pose.pose.position.y)
    ros_path[14].poses.append(PosePoint)
    publishers[14].publish(ros_path[14])

        
def main():
    global bat_num
    global path_pub, act_server_go_to_point_, rate_
    path_pub = rospy.Publisher("path",Path,queue_size=1)
    rospy.Subscriber("/bat_swarm_red/bat_1/odom", Odometry, odom_clbk1)
    rospy.Subscriber("/bat_swarm_red/bat_2/odom", Odometry, odom_clbk2)
    rospy.Subscriber("/bat_swarm_red/bat_3/odom", Odometry, odom_clbk3)
    rospy.Subscriber("/bat_swarm_red/bat_4/odom", Odometry, odom_clbk4)
    rospy.Subscriber("/bat_swarm_red/bat_5/odom", Odometry, odom_clbk5)
    rospy.Subscriber("/bat_swarm_green/bat_6/odom", Odometry, odom_clbk6)
    rospy.Subscriber("/bat_swarm_green/bat_7/odom", Odometry, odom_clbk7)
    rospy.Subscriber("/bat_swarm_green/bat_8/odom", Odometry, odom_clbk8)
    rospy.Subscriber("/bat_swarm_green/bat_9/odom", Odometry, odom_clbk9)
    rospy.Subscriber("/bat_swarm_green/bat_10/odom", Odometry, odom_clbk10)
    rospy.Subscriber("/bat_swarm_blue/bat_11/odom", Odometry, odom_clbk11)
    rospy.Subscriber("/bat_swarm_blue/bat_12/odom", Odometry, odom_clbk12)
    rospy.Subscriber("/bat_swarm_blue/bat_13/odom", Odometry, odom_clbk13)
    rospy.Subscriber("/bat_swarm_blue/bat_14/odom", Odometry, odom_clbk14)
    rospy.Subscriber("/bat_swarm_blue/bat_15/odom", Odometry, odom_clbk15)
    #for i in range(1,16):
    #    bat_num=i
    #    rospy.Subscriber(getTopicOdom(i), Odometry, odom_clbk)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('publishPaths')
    main()