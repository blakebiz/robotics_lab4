#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool


img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")
pause_toggle = False


# topic /pause_toggle
def pause_callback(data):
	global pause_toggle
	pause_toggle = data


# topic /camera/color/image_raw
def image_callback(ros_img):
	# allow access to reassign global variables
	global rgb_img
	global img_received
	# convert image to cv2 image and store it
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	img_received=True
	

def filter_image(image):
	# convert the image to the HSV space
	hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
	# define the upper and lower ranges
	lower_yellow_hsv = np.array([25,0,1])
	upper_yellow_hsv = np.array([60,255,255])
	# filter the image 
	yellow_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
	return yellow_mask


if __name__ == '__main__':
	# initialize ros node
	rospy.init_node('image_input', anonymous=True)
	# declare publisher and subscriber
	img_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size=1)
	
	# set the frequency
	rate = rospy.Rate(10)
	
	# declare array of 0's with the size of the image
	rect = np.zeros((720, 1280), dtype="uint8")
	# select a rectangle from the array
	cv2.rectangle(rect, (500, 200), (700, 450), (255, 255, 255), -1)
	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		# also check if we've been signalled to pause
		if img_received and not pause_toggle:
			# filter original image to only contain yellows in a given range
			altered_img = filter_image(rgb_img)
			# logical and to get values only in the area we want
			altered_img = cv2.bitwise_and(rect, altered_img)
			# convert back to an image ros can use
			img_msg = CvBridge().cv2_to_imgmsg(altered_img, encoding='mono8')
			# publish image to /ball_2D for final output
			img_pub.publish(img_msg)
		rate.sleep()
	
