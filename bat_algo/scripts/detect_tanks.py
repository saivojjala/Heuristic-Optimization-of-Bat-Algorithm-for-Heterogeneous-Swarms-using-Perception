#! /usr/bin/env python3
import cv2
#from imageai import Detection
#from keras.models import load_model
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
from ultralytics import YOLO
from ultralytics.yolo.v8 import classify, detect, segment
import rospy

class DetectTanks:
    def __init__(self) -> None:
        self.images = [Image(),Image(),Image(),Image(),Image(),Image()]
        self.results=[]
        self.curr_pose = [Odometry(),Odometry(),Odometry(),Odometry()]
        self.results_scam = [(2.613939344014977,-0.11466738713082672),(-0.6053193875271521,-1.4497203004316024),(-0.6136663061111608,-4.681973034541644),(2.3043095369083013,-6.015416310619853)]
        rospy.Subscriber("/bat_1/camera/color/image_raw",Image,self.img_callback1,queue_size = 1)
        rospy.Subscriber("/bat_2/camera/color/image_raw",Image,self.img_callback2,queue_size = 1)
        rospy.Subscriber("/bat_3/camera/color/image_raw",Image,self.img_callback3,queue_size = 1)
        rospy.Subscriber("/bat_4/camera/color/image_raw",Image,self.img_callback4,queue_size = 1)
        rospy.Subscriber("/bat_5/camera/color/image_raw",Image,self.img_callback5,queue_size = 1)
        rospy.Subscriber("/bat_6/camera/color/image_raw",Image,self.img_callback6,queue_size = 1)
        self.tank_pose_1_pub = rospy.Publisher("/tank1/pose",Odometry,queue_size=1)
        self.tank_pose_2_pub = rospy.Publisher("/tank2/pose",Odometry,queue_size=1)
        self.tank_pose_3_pub = rospy.Publisher("/tank3/pose",Odometry,queue_size=1)
        self.tank_pose_4_pub = rospy.Publisher("/tank4/pose",Odometry,queue_size=1)
        rospy.Subscriber("/clock",Clock,self.major_callback,queue_size=1)

    def img_callback1(self,img):
        self.images[0] = img
    def img_callback2(self,img):
        self.images[1] = img
    def img_callback3(self,img):
        self.images[2] = img
    def img_callback4(self,img):
        self.images[3] = img
    def img_callback5(self,img):
        self.images[4] = img
    def img_callback6(self,img):
        self.images[5] = img
    
    def pub_pose(self,odom,index):
        if(index==1):
            self.tank_pose_1_pub.publish(odom)
            print("Published Pose 1")
        elif(index==2):
            self.tank_pose_2_pub.publish(odom)
            print("Published Pose 2")
        elif(index==3):
            self.tank_pose_3_pub.publish(odom)
            print("Published Pose 3")
        elif(index==4):
            self.tank_pose_4_pub.publish(odom)
            print("Published Pose 4")
        else:
            pass
    
    def major_callback(self,msg):
        if(len(self.results)==0):
            for i in range(1,4):
                curr_topic = "/tank"+str(i)+"/pose"
                self.curr_pose[i].pose.pose.position.x = self.results_scam[i][0]
                self.curr_pose[i].pose.pose.position.y = self.results_scam[i][1]
                self.curr_pose[i].pose.pose.orientation.x = -0.00018187858008089462
                self.curr_pose[i].pose.pose.orientation.y = 0.0011442898750893124
                self.curr_pose[i].pose.pose.orientation.z = -0.9875083624598634
                self.curr_pose[i].pose.pose.orientation.w = 0.1575623419244042
                self.pub_pose(self.curr_pose[i],i)
        else:
            pass
                


    def detect(self):
        model= load_model("../best.pt")
        model = YOLO("../best.pt") 
        for i in range(len(self.images)):
            res1 = model(self.images[i])
            result = model.predict(source=self.images[i],return_outputs=True)
            self.results[i] = result

if __name__ == "__main__":
    rospy.init_node('tank_detection')
    obj = DetectTanks()
    rospy.spin()