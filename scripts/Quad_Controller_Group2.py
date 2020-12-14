"""
ELE/MCE 456 - Found. of Robotics
Final Project: Quadcoptor Following Ground Robot

Group Number 2:

Authors:
Matthew Morgan
Austin Clark
Nataly Cruz
"""

### --- Libraries Used --- ###
import rospy
from std_msgs.msg import Empty
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image ## NEW
from nav_msgs.msg import Odometry  ## NEW
from cv_bridge import CvBridge ## NEW

import cv2         ## opencv
import numpy as np ## NEW
import array
import math
import time

### --- Global Veriables --- ###
cmd_pub = rospy.Publisher("/drone/cmd_vel", Twist, queue_size=1)
error_integral = 0.0
error_previous = 0.0

robot_x = 0.0  ## NEW
robot_y =  0.0  ## NEW 

desired_z = 20
desired_y = 0.0
desired_x = 1.0

### --- camera parameters --- ###

af = 320.255 
Ix = 640 
Iy = 360

Ix0 = Ix/2
Iy0 = Iy/2

### --- coordinates used for ground robot --- ###

X = 0
Y = 0

cX = 0
cY = 0

### --- aerial drone coordinates --- ###

drone_x = 0.0  
drone_y =  0.0 
drone_z = 0.0 
drone_yaw = 0.0

# For Converting ROS Image to OpenCV Image
bridge = CvBridge() 

# this is our callback function: it is executed any time a message on the specified topic
# is received. In our case, the specified topic is /drone/gt_pose.
# The message received will be available in the function through the variable msg  

### --- callback function --- ###
def poseCallback(msg):
    time.sleep(0.05) #slows continuous computation

    global drone_x; drone_x = msg.position.x 
    global drone_y; drone_y = msg.position.y 
    global drone_z; drone_z = msg.position.z
    global drone_yaw; drone_yaw = msg.orientation.z  #.w or .z for yaw?
 
    # to use a global variable in a function, you must explicitly
    # redefine with the global attribute
    global cmd_pub 
    global error_integral 
    global error_previous 


    global robot_x ## NEW
    global robot_y  ## NEW

    global cX
    global cY
    global X
    global Y

    global desired_z
    global desired_y
    global desired_x

    ### ---- Frame manipulation math below ---- ###
    
    if (cX != 0.0 or cY != 0.0):
        print('--------------------------------------------')
        R_wq = np.array([[math.cos(drone_yaw), -(math.sin(drone_yaw)), 0.0],
                        [math.sin(drone_yaw), math.cos(drone_yaw), 0.0],
                        [0.0, 0.0, 1.0]])      

        #quadrotor in world frame 
        print('')
        print(R_wq)
        print('R_wq')

        R_qc = np.array([[0.0, -0.98545, 0.16997],[-1.0, 0.0, 0.0],[0.0, -0.1699, -0.98545]])
        #R_qc = np.array([[-0.9855, 0.0, 0.1698],[0.0, 1.000, 0.0],[-0.1698, 0.0, -0.9855]])  
        #camera in quadrotor frame
        print('')
        print(R_qc)
        print('R_qc')

        R_wc = R_wq.dot(R_qc) 
        print('')
        print(R_wc)
        print('R_wc')
        #camera in world frame

        P_wq = np.array([[drone_x], [drone_y], [drone_z]])
        print('')
        print(P_wq)
        print('P_wq')

        T = np.concatenate((R_wc,P_wq), axis = 1)     
        #intermediate step to compute homogenous transform matrix
        print('')
        print(T)
        print('T')

        T1 = np.array([0.0, 0.0, 0.0, 1.0])
        #intermediate step to compute homogenous transform matrix
        print(' ')
        print(T1)
        print('T1')


        T_wc = np.vstack((T,T1)) #homogenous transform matrix
        print(' ')
        print(T_wc)
        print('T_wc')

        P_ct = np.array([[X],[Y],[20.0],[1.0]]) 
        #Target array in camera frame
        print(' ')
        print(P_ct)
        print('P_ct')


        P_wt = T_wc.dot(P_ct)
         #Target array in world frame ******
        print(' ')
        print(P_wt)
        print('P_wt')
        ##Coordinates of robot determined by camera in world frame
        #robot_x = P_wt[1]    
        #robot_y = P_wt[2]
        
        print('--------------------------------------------')
    
    print(" target x pix ", cX, " target y pix ", cY) ## open image coordinates

    print('')
    print(" position x ", drone_x)
    print(" position y ", drone_y)
    print(" position z ", drone_z)
    print(" yaw z ", drone_yaw)
    print('')


    # Issue position from current location of drone in frame
    # Pass parameters to PID controllers 
    if (cX != 0.0 or cY != 0.0):
        desired_y = robot_y        #Y
        desired_x = robot_x - 1    #X
        if cY > 100 and cY < 300 and desired_z >= 2:
            desired_z = msg.position.z   #slow the drone
            desired_z = desired_z - 1
        else:
            desired_z = msg.position.z   #slow the drone
            desired_z = desired_z + 0.5
       
    # z controller - up and down
    error_z = desired_z - msg.position.z
    linear_velocity_z = 1*error_z 

    # y controller - side to side
    error_y = desired_y - msg.position.y
    linear_velocity_y = 0.2*error_y
    
    # x controller - forward and backward
    error_x = desired_x - msg.position.x
    linear_velocity_x = 0.2*error_x


    #cmd_pub can be issuing multiple velocity commands at once?    
    cmdmsg = Twist()
    
    cmdmsg.linear.x = linear_velocity_x 
    cmdmsg.linear.y = linear_velocity_y 
    cmdmsg.linear.z = linear_velocity_z

    cmd_pub.publish(cmdmsg)



### --- function for imaging data --- ###

def imageCallback(msg):
    
    global af
    global Ix0
    global Iy0 
    global bridge
    global cX
    global cY
    global X
    global Y


    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') #Convert ROS Image to OpenCV Image


    lower = np.array([240,240,240], dtype = "uint8")  ## Lower color bound (Off white)
    upper = np.array([255,255,255], dtype = "uint8")  ## Upper color bound (White)

    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) #returns grayscale image 

    ### ------------------------------------------------- ###

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,230,255,0)
	# calculate moments of binary image
    M = cv2.moments(thresh)

	# calculate x,y coordinate of center of geometry (in pixels)
    if M["m00"] != 0:
	    cX = int(M["m10"] / M["m00"])
	    cY = int(M["m01"] / M["m00"])
    else:
	    cX, cY = 0, 0

    X = ((cX-Ix0)*drone_z)/af
    Y = ((cY-Iy0)*drone_z)/af

    # put text and highlight the center
    cv2.circle(cv_image, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(cv_image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # display the image
    cv2.imshow("Image", cv_image)
    cv2.waitKey(3)

    ### ---------different image outputs from Opencv--------- ###

    mask = cv2.inRange(cv_image, lower, upper) #returns binary mask that fall within bounds
    output = cv2.bitwise_and(cv_image, cv_image, mask = mask) #computes underlying binary representation of integers in input array

    cv2.imshow("Image window - Grayscale", gray_image) #Create Window to show the image (grayscale)
    #cv2.imshow("Image window - Raw", cv_image) #Create Window to show the image
    #cv2.imshow("Image window - OpenCV", output) #Create Window to show the image
    cv2.waitKey(3) #Need to wait for window to load



# Fucntion not used to give positional data to aerial drone
# Use for reference def odomCallback(msg)
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