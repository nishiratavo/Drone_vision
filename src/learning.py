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
from math import *
from waypoint import waypoint_mode
from ardrone_autonomy.msg import Navdata

mode = -1
change_mode = 0
image_exist = 0

keyboard_mode = rospy.Publisher("data", Quaternion, queue_size = 10)

class image_receiver:
	''' Class used in the modes in which camera is required '''

	def __init__(self, camera = -1) :
		self.best_x = 0
		self.camera = camera
		self.bridge = CvBridge()
		self.subscribed = 0
		#self.lower = np.array([0, 0, 70], dtype = "uint8")
		#self.upper = np.array([40, 40, 255], dtype = "uint8")
		self.lower = np.array([200, 240, 200], dtype = "uint8")
		self.upper = np.array([255, 255, 255], dtype = "uint8")
		self.contours = []
		self.kernelOpen=np.ones((5,5))
		self.kernelClose=np.ones((20,20))
		#self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)
		#self.image_sub.unregister()
		self.image_pos_pub = rospy.Publisher("data", Quaternion, queue_size = 10)
		self.teste = rospy.Publisher("teste_reg", String, queue_size=10)
		self.state = 0
		#self.mode = rospy.Subscriber("/mode", Int32, self.mode_selection)



	def color_detect(self, camera_image):
		# receives a image and return the mask, with only the chosen color
		mask = cv2.inRange(camera_image, self.lower, self.upper)
		im2, self.contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		one_color_image = cv2.bitwise_and(camera_image, camera_image, mask = mask)
		return one_color_image


	def follow_line(self, camera_image):
		# sends for the controller the distance from the line and the angle between the drone and the line
		global image_exist
		height, width = camera_image.shape[:2]
		crop_img = camera_image[0:150, 0:width]
		mask = cv2.inRange(crop_img, self.lower, self.upper)
		im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		one_color_image = cv2.bitwise_and(crop_img, crop_img, mask = mask)

		best_x, best_y = width/2, height + 1
		worst_x, worst_y = width/2, -1
		detecting = 0
		for i in contours:
			detecting = 1
			(x,y), radius = cv2.minEnclosingCircle(i)
			if y < best_y:
				best_y = y
				best_x = x
			if y > worst_y:
				worst_y = y
				worst_x = x
		if best_y != worst_y:
			ang = float(-((best_x - worst_x)/(best_y - worst_y)))
		else :
			ang = 0
		self.worst_x = (worst_x - width/2)
		cv2.circle(one_color_image, (int(best_x), int(best_y)), 5,(0,255,0),-1)
		cv2.circle(one_color_image, (int(worst_x), int(worst_y)), 5,(0,255,0),-1)
		self.image_pos_pub.publish(self.worst_x,detecting,atan(ang),self.camera)
		one_color_image = cv2.resize(one_color_image, (0,0), fx = 0.7, fy = 0.7)
		crop_img = cv2.resize(crop_img, (0,0), fx = 0.7, fy = 0.7)
		cv2.imshow("Image window", np.hstack([one_color_image,crop_img]))
		image_exist = 1
		cv2.waitKey(3)



	def callback(self,data):
		# selects between modes
		if True:

			self.teste.publish("Entrou")
			global image_exist
			if self.camera == -1:
				call(["rosservice", "call", "ardrone/setcamchannel", "0"])
				self.camera = 0

			if self.camera == 1:
				call(["rosservice", "call", "ardrone/setcamchannel", "1"])
				self.camera = 2

			if self.camera == 3:
				call(["rosservice", "call", "ardrone/setcamchannel", "1"])
				self.camera = 4
			
			start = time.time()
			try:
				cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
			except CvBridgeError as e:
				print (e);
			
			if self.camera == 0 or self.camera == 2:
			   	self.camera_track(cv_image)
			else:
				self.follow_line(cv_image)	

	def camera_track(self, cv_image):
		# sends for the controller the error in the 2 axis
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

		output_image = cv2.resize(output_image, (0,0), fx = 0.7, fy = 0.7)
		cv_image = cv2.resize(cv_image, (0,0), fx = 0.7, fy = 0.7)
		cv2.imshow("Image window", np.hstack([output_image, cv_image]))
		image_exist = 1
		cv2.waitKey(3)



	def unsub(self):
		self.image_sub.unregister()

	def sub(self):
		self.image_sub = rospy.Subscriber("/ardrone/image_raw",Image,self.callback)

	def __del__(self):
		pass



ic = None
way = None
ic = image_receiver(-1)
way = waypoint_mode()





def mode_selection(data):

	global mode
	global ic
	global image_exist
	global way
	mode = data.data

	if mode == 1:
		ic.camera = -1

	elif mode == 2:
		ic.camera = 1

	elif mode == 3:
		ic.camera = 3

	else:
		cv2.destroyAllWindows()





def callback(data):
	if ((mode == 1) or (mode == 2) or (mode == 3)):
		ic.callback(data)
	if mode == 5:
		keyboard_mode.publish(0,0,0,6)


def ReceiveNavdata(data):
	if mode == 4:
		way.ReceiveNavdata(data)



def main(args):
	rospy.init_node('image_receiver', anonymous=True)
	rospy.Subscriber("/mode", Int32, mode_selection)
	rospy.Subscriber("/ardrone/image_raw",Image,callback)
	rospy.Subscriber('/ardrone/navdata',Navdata,ReceiveNavdata)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)

