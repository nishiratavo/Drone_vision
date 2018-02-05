#!/usr/bin/env python

import roslib
roslib.load_manifest('Drone_vision')
import sys
import rospy
import numpy as np
import time
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from subprocess import call
from math import *
from ardrone_autonomy.msg import Navdata



class waypoint_mode:

	def __init__(self):
		self.points = []
		self.dx = 0
		self.dy = 0
		self.dz = 0
		self.last_vx = 0
		self.last_vy = 0
		self.last_vz = 0
		self.last_time = 0
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
		self.error_pub = rospy.Publisher("data", Quaternion, queue_size = 10)
		self.reset = rospy.Subscriber("/position", Int32, self.Reset)
		self.get_commands = rospy.Subscriber("/waypoint_receiver", String , self.SetWaypoint)



	def ReceiveNavdata(self,navdata):
		if self.points != []:
			vx = navdata.vy/1000.0
			vy = navdata.vx/1000.0
			vz = navdata.vz/1000.0
			actual_time = time.time()
			dt = actual_time - self.last_time
			delta_x = (vx + self.last_vx)/2
			self.points[0][0] -= delta_x*dt
			delta_y = (vy + self.last_vy)/2
			self.points[0][1] -= delta_y*dt
			delta_z = (vz + self.last_vz)/2
			self.points[0][2] -= delta_z*dt
			self.last_vx = vx
			self.last_vy = vy
			self.last_vz = vz
			self.last_time = actual_time
			if abs(self.points[0][0]) < 0.1:
				roll = 0
			elif self.points[0][0] > 0:
				roll = 1
			else:
				roll = -1

			if abs(self.points[0][1]) < 0.1:
				pitch = 0
			elif self.points[0][1] > 0:
				pitch = 1
			else:
				pitch = -1

			if abs(self.points[0][2]) < 0.1:
				altitude = 0
			elif self.points[0][2] > 0:
				altitude = 1
			else:
				altitude = -1
			self.error_pub.publish(self.points[0][0],self.points[0][1],self.points[0][2],5)
			if ((abs(self.points[0][1]) < 0.1) and (abs(self.points[0][1]) < 0.1) and (abs(self.points[0][1]) < 0.1)):
				del self.points[0]
		else:
			self.error_pub.publish(0,0,0,5)

	def SetWaypoint(self,data):
		if not(type(data) is str):
			data = data.data
		
		deltas = data.split(",")
		dx = float(deltas[0])
		dy = float(deltas[1])
		dz = float(deltas[2])
		self.points.append([dx,dy,dz])
		#self.last_time = time.time()

	def Reset(self,data):
		if self.points == []:
			self.points.append([0,0,0])
		else:
			self.points[0] = [0,0,0]





