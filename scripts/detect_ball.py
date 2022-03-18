#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


# topic /camera/color/image_raw
def image_callback(data):
	global rgb_img
	global img_received
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	img_received=True
	

def filter_image(image):
	# convert the image to the HSV space
	hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
	# define the upper and lower ranges
	lower_yellow_hsv = np.array([25,100,1])
	upper_yellow_hsv = np.array([60,255,255])
	# filter the image 
	yellow_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
	return yellow_mask


if __name__ == '__main__':
	rospy.init_node('image_input', anonymous=True)
	img_sub = rospy.Supscriber('/camera/color/image_raw', Image, image_callback)
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size=1)
	
	# set the frequency
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			altered_img = filter_image(rgb_img)
			img_msg = CvBridge().cv2_to_imgmsg(altered_img, encoding='rgb8')
			img_pub.publish(img_msg)
		rate.sleep()
	
