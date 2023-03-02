#! /usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import random
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import numpy as np
import time
from scipy.spatial import ConvexHull, convex_hull_plot_2d
import numpy as np
from scipy.spatial import KDTree 

class Coordinate:
    def __init__(self,x,y) -> None:
        self.x = x
        self.y = y
    def setX(self,x):
        self.x = x
    def setY(self,y):
        self.y = y


class GetGlobalPoints:
    def __init__(self) -> None:
        self.detected_points=[]
        self.hull =[]

    def setPoints(self,points):
        self.detected_points = points

    def addPoint(self,point):
        self.detected_points.append(point)
    def createConvexHull(self):
        self.hull = ConvexHull(self.detected_points)

    def plotHull(self):
        print(self.hull.equations)
        plt.plot(self.detected_points[:,0], self.detected_points[:,1], 'o')
        for simplex in self.hull.simplices:
            plt.plot(self.detected_points[simplex, 0], self.detected_points[simplex, 1], 'k-')
        plt.show()
    
class PublishPoints:
    def __init__(self) -> None:
        self.points = []
        self.mirror_points = []
        self.hullobj = GetGlobalPoints()
        self.pose_1_pub = rospy.Publisher("/global_pose_1",Odometry,queue_size=1)
        self.pose_2_pub = rospy.Publisher("/global_pose_2",Odometry,queue_size=1)
        self.pose_3_pub = rospy.Publisher("/global_pose_3",Odometry,queue_size=1)
        self.pose_4_pub = rospy.Publisher("/global_pose_4",Odometry,queue_size=1)
        rospy.Subscriber("/tank1/pose",Odometry,self.odom_callback1,queue_size = 1)
        rospy.Subscriber("/tank2/pose",Odometry,self.odom_callback2,queue_size = 1)
        rospy.Subscriber("/tank3/pose",Odometry,self.odom_callback3,queue_size = 1)
        rospy.Subscriber("/tank4/pose",Odometry,self.odom_callback4,queue_size = 1)
        rospy.Subscriber("/odom",Odometry,self.major_callback,queue_size=1)
    
    def odom_callback1(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if(x!=255 and y!=255):
            self.points[0] = [x,y]
        else:
            pass

    def odom_callback2(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if(x!=255 and y!=255):
            self.points[1] = [x,y]
        else:
            pass

    def odom_callback3(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if(x!=255 and y!=255):
            self.points[2] = [x,y]
        else:
            pass

    def odom_callback4(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if(x!=255 and y!=255):
            self.points[3] = [x,y]
        else:
            pass
    
    def major_callback(self):
        self.hullobj.createConvexHull()
        


if __name__ == "__main__":
    rospy.init_node('publish_points')
    generators = np.array([[0.2, 0.2],
                       [0.2, 0.4],
                       [0.4, 0.4],
                       [0.4, 0.2],
                       [0.3, 0.6]])
    obj = GetGlobalPoints()
    obj.setPoints(generators)
    obj.createConvexHull()
    obj.plotHull()
    
"""
Point :1 
pose: 
  pose: 
    position: 
      x: -6.731236739721421
      y: -0.06402044090225052
      z: 0.025012827454624416
    orientation: 
      x: -0.001266236807593421
      y: 2.5202673662973647e-06
      z: -0.004199342110711902
      w: 0.9999903810356541

Point : 2
pose: 
  pose: 
    position: 
      x: -5.104874290975364
      y: -1.4952762161233175
      z: 0.02501317597124113
    orientation: 
      x: -0.001288503778256083
      y: 1.49828377470175e-06
      z: -0.0034090134249990306
      w: 0.9999933591695681
Point : 3
pose: 
  pose: 
    position: 
      x: -4.700912500625259
      y: -4.484871600869022
      z: 0.02501131557797427
    orientation: 
      x: -0.0015020934451765583
      y: 2.663697642936264e-06
      z: -0.0027122506409442415
      w: 0.9999951936907735
Point 4:
pose: 
  pose: 
    position: 
      x: -6.010407068227448
      y: -6.081954633384385
      z: 0.02501286678610454
    orientation: 
      x: -0.001266470465528682
      y: -7.07294728831397e-07
      z: -0.0021723947440467953
      w: 0.9999968383715699

"""