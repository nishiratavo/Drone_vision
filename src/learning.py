#!/usr/bin/env python

import roslib
roslib.load_manifest('Drone_vision')
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
		self.camera = camera
		self.bridge = CvBridge()
		#self.lower = np.array([0, 0, 70], dtype = "uint8")
		#self.upper = np.array([40, 40, 255], dtype = "uint8")
		self.lower = np.array([200, 240, 200], dtype = "uint8")
		self.upper = np.array([255, 255, 255], dtype = "uint8")
		self.contours = []
		self.kernelOpen=np.ones((5,5))
		self.kernelClose=np.ones((20,20))
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
		self.image_pos_pub = rospy.Publisher("data", Quaternion, queue_size = 10)
		self.state = 0



	def color_detect(self, camera_image):
		mask = cv2.inRange(camera_image, self.lower, self.upper)
		#maskOpen=cv2.morphologyEx(mask,cv2.MORPH_OPEN,self.kernelOpen)
		#maskClose=cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,self.kernelClose)
		im2, self.contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		one_color_image = cv2.bitwise_and(camera_image, camera_image, mask = mask)
		return one_color_image


	def follow_line(self, camera_image):
		height, width = camera_image.shape[:2]
		crop_img = camera_image[int(height/2 - 50):int(height/2 + 50), 0:width]
		mask = cv2.inRange(crop_img, self.lower, self.upper)
		im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		one_color_image = cv2.bitwise_and(crop_img, crop_img, mask = mask)

		best_x, best_y = width/2, height + 1
		for i in contours:
			(x,y), radius = cv2.minEnclosingCircle(i)
			if y < best_y:
				best_y = y
				best_x = x
		cv2.circle(one_color_image, (int(best_x), int(best_y)), 5,(0,255,0),-1)
		self.image_pos_pub.publish(best_x-width/2,0,0,self.camera)
		#cv2.drawContours(one_color_im)age, contours, -1, (0,255,0), 3)
		cv2.imshow("Image window", np.hstack([one_color_image,crop_img]))
		cv2.waitKey(3)



	def callback(self,data) :
		if self.camera == 1:
			call(["rosservice", "call", "ardrone/togglecam"])
			self.camera = 2

		if self.camera == 3:
			call(["rosservice", "call", "ardrone/togglecam"])
			self.camera = 4

		start = time.time()
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print (e);
		
		if self.camera == 0 or self.camera == 2:
		   	output_image = self.color_detect(cv_image)
			for i in self.contours:
				(x,y),radius = cv2.minEnclosingCircle(i)
				if radius < 20:

					center = (int(x),int(y))
					radius = int(radius)
					cv2.circle(output_image,center,radius,(0,255,0),2)
					height, width = output_image.shape[:2]
					x = int(x - width/2)
					y = int(y - height/2)
					self.image_pos_pub.publish(x,y,radius,self.camera)
					break
				else :
					self.image_pos_pub.publish(0,0,0,self.camera)
			
			if self.contours == []:
				self.image_pos_pub.publish(0,0,0,self.camera)

			cv2.imshow("Image window", np.hstack([output_image, cv_image]))
			cv2.waitKey(3)
		else:
			self.follow_line(cv_image)
		



def main(args):
	ic = image_receiver(0)
	rospy.init_node('image_receiver', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)

