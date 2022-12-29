#!/usr/bin/env python3 
import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
import cv2
import numpy as np

bridge_object = CvBridge() # create the cv_bridge object 
image_received = 0 #Flag to indicate that we have already received an image 
cv_image = 0 #This is just to create the global variable cv_image  

def show_image(): 
    image_sub = rospy.Subscriber("/thermal_camera/image_raw",Image,camera_callback) 
    r = rospy.Rate(10) #10Hz  
    while not rospy.is_shutdown():  
        if image_received: 
            cv2.waitKey(1) 
            r.sleep()  
    cv2.destroyAllWindows() 
    
def process_image(image):
        image = cv2.resize(image,(160,120)) 
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 

        min_green = np.array([50,220,220]) 
        max_green = np.array([60,255,255]) 
        min_red = np.array([170,220,220]) 
        max_red = np.array([180,255,255]) 
        min_blue = np.array([110,220,220]) 
        max_blue = np.array([120,255,255]) 
        
        mask_g = cv2.inRange(hsv, min_green, max_green) 
        mask_r = cv2.inRange(hsv, min_red, max_red) 
        mask_b = cv2.inRange(hsv, min_blue, max_blue) 

        res_b = cv2.bitwise_and(image, image, mask= mask_b) 
        res_g = cv2.bitwise_and(image,image, mask= mask_g) 
        res_r = cv2.bitwise_and(image,image, mask= mask_r) 
        cv2.imshow('Green',res_g) 
        cv2.imshow('Red',res_b) 
        cv2.imshow('Blue',res_r) 
        cv2.imshow('Original',image) 
        cv2.waitKey(1) 
        
def camera_callback(data): 
    global bridge_object 
    global cv_image 
    global image_received 
    image_received=1 
    try: 
        print("received ROS image, I will convert it to opencv") 
        # We select bgr8 because its the OpenCV encoding by default 
        cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") 
        #Add your code to save the image here: 
        #Save the image "img" in the current path  
        cv2.imwrite('robot_image.jpg', cv_image)        
        ## Calling the processing function
        process_image(cv_image)
        cv2.imshow('Image from robot camera', cv_image) 
    except CvBridgeError as e: 
        print(e) 
    
if __name__ == '__main__': 
    rospy.init_node('load_image', anonymous=True) 
    show_image() 

