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
from pid import PID



class waypoint_mode:

	def __init__(self):
		self.points = []
		#self.dx = 0
		#self.dy = 0
		#self.dz = 0
		self.last_vx = 0
		self.last_vy = 0
		#self.last_vz = 0
		self.last_time = 0
		self.new_point = 0
		self.yaw_offset = 0
		self.offset = 0
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
		self.error_pub = rospy.Publisher("data", Quaternion, queue_size = 10)
		self.reset = rospy.Subscriber("/position", Int32, self.Reset)
		self.get_commands = rospy.Subscriber("/waypoint_receiver", String , self.SetWaypoint)
		self.teste = rospy.Publisher("/teste", String, queue_size=10)
		self.roll_control = PID(1,0,1)
		self.altitude_control = PID(1,0,0)
		self.pitch_control = PID(1,0,1)
		#self.yaw_control = PID(0.1,0,0.5)



	def ReceiveNavdata(self,navdata):
		if self.points != []:
			vx = navdata.vy/1000.0
			vy = navdata.vx/1000.0
			#vz = navdata.vz/1000.0
			z = navdata.altd/1000.0
			'''if self.yaw_offset == 0:
				self.offset = navdata.rotZ
				self.yaw_offset = 1'''

			if (self.new_point == 1) and (z != 0) and (z > 0.6):
				self.points[0][2] += z
				if self.points[0][2] < 0.5:
					self.Reset(1)
					return None

				self.new_point = 0

			actual_time = time.time()
			dt = actual_time - self.last_time
			delta_x = (vx + self.last_vx)/2
			self.points[0][0] -= delta_x*dt
			delta_y = (vy + self.last_vy)/2
			self.points[0][1] -= delta_y*dt
			error_z = self.points[0][2] - z
			#error_yaw = -0.5*(navdata.rotZ - self.offset)
			self.last_vx = vx
			self.last_vy = vy
			self.last_time = actual_time
			if abs(self.points[0][0]) < 0.1:
				roll = 0
			else:
				roll = self.roll_control.update(self.points[0][0])
			'''elif self.points[0][0] > 0:
				roll = min(1, self.points[0][0])
			else:
				roll = max(-1, self.points[0][0])'''

			if abs(self.points[0][1]) < 0.1:
				pitch = 0
			else:
				pitch = self.pitch_control.update(self.points[0][1])
		

			if abs(error_z) < 0.1:
				altitude = 0
			else:
				altitude = self.altitude_control.update(error_z)

			#yaw = self.yaw_control.update(error_yaw)
			self.error_pub.publish(roll,pitch,error_z,5)
			#self.teste.publish(str(roll) + " " + str(pitch) + " " + str(altitude))
			self.teste.publish(str(self.points[0][0]) + " " + str(self.points[0][1]) + " " + str(error_z) + " " + str(self.points[0][2]) + " " + str(z))
			if ((abs(self.points[0][0]) < 0.1) and (abs(self.points[0][1]) < 0.1) and (abs(error_z) < 0.1)):
				del self.points[0]
				self.new_point = 1
				self.error_pub.publish(0,0,0,5)
				time.sleep(1)
				self.last_time = time.time()
		else:
			self.error_pub.publish(0,0,0,5)
			self.last_time = time.time()
			self.teste.publish("vazio")
		#self.teste.publish(str(self.points) + " " + str(pitch) + " " + str(altitude))

	def SetWaypoint(self,data):
		self.new_point = 1
		if not(type(data) is str):
			data = data.data
		points = data.split(" ")
		for data in points:
			deltas = data.split(",")
			dx = float(deltas[0])
			dy = float(deltas[1])
			dz = float(deltas[2])
			self.points.append([dx,dy,dz])
		#self.last_time = time.time()

	def Reset(self,data):
		self.points = [[0,0,0]]





