#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge,CvBridgeError
import sys

bridge__=CvBridge()

name_space_ = rospy.get_namespace()

def get_contour_center(contour):
    M=cv2.moments(contour)
    cx=-1
    cy=-1
    if(M['m00']!=0):
        cx=int(M['m10']/M['m00'])
        cy=int(M['m01']/M['m00'])
    return cx,cy

def image_callback(ros_image):
    # print('got an image')
    global bridge_
    try:
        image=bridge_.imgmsg_to_cv2(ros_image,"bgr8")
        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        # cv2.imshow("image",image)
        # cv2.imshow("hsv", hsv)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    except CvBridgeError as e:
        rospy.logerr(msg="CvBridgeError: [%s]" %(e))
    redLower=(0,125,50)
    redUpper=(200,255,255)
    mask=cv2.inRange(hsv,redLower,redUpper)
    # gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    #mask=cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,5,2)
    #_,mask=cv2.threshold(gray,127,255,cv2.THRESH_BINARY)
    # cv2.imshow("mask",mask)
    contours, hierarchy= cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        ((x,y),radius)= cv2.minEnclosingCircle(c)
        if (area>50):
            # cv2.drawContours(image,[c],-1,(150,250,150),2)
            # cv2.drawContours(mask,[c],-1,(150,250,150),2)
            cx, cy = get_contour_center(c)
            # cx ~ 0->160 | 80
            # cy ~ 0->120 | 60
            # cv2.circle(image, (cx,cy), (int)(radius), (0,0,255),3)
            #cv2.circle(mask, (cx,cy), (int)(radius), (0,0,255),3)
            #cv2.circle(mask, (cx,cy), 5, (0,0,255),-1)
            rospy.loginfo("Area: %s,Perimeter: %s,\tCx: %s, Cy: %s" %(area,perimeter,cx,cy))
            # print("number of contours:{}".format(len(contours)))
            cv2.imshow("RGB", image)
            # cv2.imshow("BW", mask)
            key=cv2.waitKey(70) & 0xFF
            if(key==ord('q')):
                cv2.destroyAllWindows()
        else:
            print('too small area')

def main(args):
    rospy.init_node('image_converter',anonymous=True)
    image_sub=rospy.Subscriber("/thermal_camera/image_raw", Image,image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        # print("Shutting down")
        cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)