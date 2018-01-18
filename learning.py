#!/usr/bin/env python

import roslib
roslib.load_manifest('learning')
import sys
import rospy
import numpy as np
import cv2
import time
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from subprocess import call


class image_receiver:

	def __init__(self, camera = 0) :
		#self.cascade = cv2.CascadeClassifier("/home/gustavo/Desktop/haarcascade_frontalface_alt.xml")
		self.camera = camera
		self.bridge = CvBridge()
		self.lower = np.array([0, 0, 70], dtype = "uint8")
		self.upper = np.array([40, 40, 255], dtype = "uint8")
		self.contours = []
		self.kernelOpen=np.ones((5,5))
		self.kernelClose=np.ones((20,20))
    	#self.cascade = cv2.CascadeClassifier("~/Desktop/haarcascade_frontalface_alt.xml")
    	#rects = cascade.detectMultiScale(img, 1.3, 4, cv2.cv.CV_HAAR_SCALE_IMAGE, (20,20))
		#call(["rosservice", "call", "ardrone/setcamchannel", "0"])
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
		self.image_pos_pub = rospy.Publisher("data", Quaternion, queue_size = 10)
		self.state = 0
		#call(["rosservice", "call", "ardrone/setcamchannel", "1"])
		#rospy.wait_for_service('ardrone/togglecam')
		#toggle = rospy.ServiceProxy('ardrone/togglecam', uint8)
		#try:
		#	toggle()
		#except rospy.ServiceException as exc:
		#	print("Service did not process request: " + str(exc))


		self.x = 0
		self.y = 0

	def detect(self, camera_image):
	    #img = cv2.imread(camera_image)
	    rects = self.cascade.detectMultiScale(camera_image, 1.3, 4, cv2.CASCADE_SCALE_IMAGE, (20,20))

	    if len(rects) == 0:
	        return [], camera_image
	    rects[:, 2:] += rects[:, :2]
	    return rects, camera_image 

	def box(self, rects, img):
	    for x1, y1, x2, y2 in rects:
	        cv2.rectangle(img, (x1, y1), (x2, y2), (127, 255, 0), 2)
	    cv2.imshow("Image window", img);

	def color_detect(self, camera_image):
		mask = cv2.inRange(camera_image, self.lower, self.upper)
		maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,self.kernelOpen)
		maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,self.kernelClose)
		im2, self.contours, hierarchy = cv2.findContours(maskClose, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		#im2, self.contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		output = cv2.bitwise_and(camera_image, camera_image, mask = maskClose)
		#maskOpen=cv2.morphologyEx(output,cv2.MORPH_OPEN,self.kernelOpen)
		#maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,self.kernelClose)
		#im2, self.contours, hierarchy = cv2.findContours(maskClose, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return output



	def callback(self,data) :
		if self.camera == 1:
			call(["rosservice", "call", "ardrone/togglecam"])
			self.camera = 2

		start = time.time()
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print (e);

		#rects = self.cascade.detectMultiScale(cv_image, 1.3, 4, cv2.CASCADE_SCALE_IMAGE, (20,20))

	   	image = self.color_detect(cv_image)
		if len(self.contours) > 0:
			(self.x,self.y),radius = cv2.minEnclosingCircle(self.contours[0])
			center = (int(self.x),int(self.y))
			radius = int(radius)
			cv2.circle(image,center,radius,(0,255,0),2)
			height, width = image.shape[:2]
			self.x = int(self.x - width/2)
			self.y = int(self.y - height/2)
			center = (int(self.x - width/2),int(self.y - height/2))
			self.image_pos_pub.publish(self.x,self.y,radius,self.camera)
		else :
			self.image_pos_pub.publish(0,0,0,self.camera)

	   		#cv2.drawContours(image, self.contours, -1, (0,255,0), 3)

	   	#cv2.drawContours(image, self.contours, -1, (0,255,0), 3)
		cv2.imshow("Image window", np.hstack([image, cv_image]))
		cv2.waitKey(3)
		


def main(args):
	ic = image_receiver(1)
	#call(["rosservice", "call", "ardrone/setcamchannel", "1"])
	rospy.init_node('image_receiver', anonymous=True)
	#call(["rosservice", "call", "ardrone/togglecam"])
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)

