#!/usr/bin/env python
#import sys
#sys.path.append('/usr/include')  #Austin can get his life together

import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image ## NEW
from nav_msgs.msg import Odometry  ## NEW
from cv_bridge import CvBridge ## NEW

import cv2
import numpy as np ## NEW



# this is a global variable: all functions and methods in this file have access to it
cmd_pub = rospy.Publisher("/drone/cmd_vel", Twist, queue_size=1)
error_integral = 0.0
error_previous = 0.0

robot_x = 0.0  ## NEW
robot_y =  0.0  ## NEW

cX = 0
cY = 0

bridge = CvBridge() ## NEW - For Converting ROS Image to OpenCV Image


# this is our callback function: it is executed any time a message on the specified topic
# is received. In our case, the specified topic is /drone/gt_pose.
# The message received will be available in the function through the variable msg  
def poseCallback(msg):
		
    # to use a global variable in a function, you must explicitly
    # redefine with the global attribute
    global cmd_pub 
    global error_integral 
    global error_previous

    global robot_x ## NEW
    global robot_y  ## NEW

    global cX
    global cY

    print(" desired x ", robot_x, " desired y ", robot_y) ## NEW

    # here I print the z component of the position (the height of the drone)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %f", msg.position.z)

    # set the desired flight height here, and the gains of the controller
    desired_z = 20.0  #in meters
    k_p = 1.0 # proportional gain
    k_i = 0.2 # integral gain
    k_d = 0.0 # derivative gain

    # compute the error
    error = desired_z - msg.position.z
    error_integral = error_integral + error*0.001
    error_derivative = (error - error_previous)/0.001

    print(" target x ", cX, " target y ", cY) ## NEW

    # compute the command needed for the 
    linear_velocity_z = k_p*error + k_i*error_integral + k_d*error_derivative
    
    #print("error ", error, " position ", z, " command ", linear_velocity_z)
    
    # here I create a new Twist message (=linear velocity and angular velocity), I write
    # the value I computed for the linear velocity on z to achieve a flight height of 2m
    # and I publish it on the appropriate topic
    cmdmsg = Twist()
    cmdmsg.linear.z = linear_velocity_z
    # cmdmsg.linear.z = -0.1
    cmd_pub.publish(cmdmsg)


## NEWW function for imaging data
def imageCallback(msg):

    global bridge
    global cX
    global cY

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') #Convert ROS Image to OpenCV Image

    #print("aaa") #Test print

    lower = np.array([100,100,100], dtype = "uint8")  ## Lower color bound (Off white)
    upper = np.array([255,255,255], dtype = "uint8")  ## Upper color bound (White)

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) #returns grayscale image 

    ### ------------------------------------------------- ###

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,127,255,0)
	# calculate moments of binary image
    M = cv2.moments(thresh)

	# calculate x,y coordinate of center
    if M["m00"] != 0:
	    cX = int(M["m10"] / M["m00"])
	    cY = int(M["m01"] / M["m00"])
    else:
	    cX, cY = 0, 0

    #print(" target x ", cX, " target y ", cY) ## NEW

    # put text and highlight the center
    cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(cv_image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # display the image
    cv2.imshow("Image", cv_image)
    cv2.waitKey(3)

    ### ------------------------------------------------- ###

    mask = cv2.inRange(cv_image, lower, upper) #returns binary mask that fall within bounds
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask) #computes underlying binary representation of integers in input array

    cv2.imshow("Image window - Grayscale", gray_image) #Create Window to show the image (grayscale)
    #cv2.imshow("Image window - Raw", cv_image) #Create Window to show the image
    #cv2.imshow("Image window - OpenCV", output) #Create Window to show the image
    cv2.waitKey(3) #Need to wait for window to load



## NEW function for poisitonal data
# Fucntion not used to give positional data to aerial drone
# Use for referencedef odomCallback(msg)
def odomCallback(msg):
    global robot_x; global robot_y
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y



def my_first_controller():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pub = rospy.Publisher("/drone_0/cmd_vel", Empty, queue_size=1)

    rospy.init_node('my_first_controller', anonymous=False)

    rospy.Subscriber("/drone/gt_pose", Pose, poseCallback, queue_size=1)

    rospy.Subscriber("/drone/camera/image_raw", Image, imageCallback, queue_size=1) ## NEW
    rospy.Subscriber("/my_robot/odom", Odometry, odomCallback, queue_size=1)  ## NEW

    emptymsg = Empty()
    pub.publish(emptymsg)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


# this is the main function: the program starts here
if __name__ == '__main__':
    my_first_controller()