#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('Drone_vision')
import sys
import rospy
from std_msgs.msg import String, Int32
from pid import PID
from drone_status import DroneStatus
from PySide import QtCore, QtGui
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import numpy as np
from math import *

class controller:
	
	def __init__(self):

		self.roll_controll = PID(10,0.1,25)
		self.altitude_control = PID(1,0,0)
		self.pitch_control = PID(0.1,0,0)
		self.yaw_control = PID(1,0,0.1)
		self.command = Twist()

		self.state_altitude = 0
		self.status = -1
		self.takeoff_time = 0
		self.image_pos = rospy.Subscriber("data",Quaternion,self.callback)
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
		self.receive_key = rospy.Subscriber('/keyboard', Int32, self.keyPressEvent) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)
		self.pub_test = rospy.Publisher('/pid_teste',String, queue_size=10)


	def ReceiveNavdata(self,navdata):
		# gets navdata state	
		self.status = navdata.state
		self.rotX = navdata.rotX*(pi/180)



	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())
			

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self):
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)


	def keyPressEvent(self,data):
		if data.data  == 1 :
			self.roll_controll.reset()
			self.altitude_control.reset()
			self.pitch_control.reset()
			self.yaw_control.reset()
			self.takeoff_time = time.time()
			self.SendTakeoff()
		elif data.data == 2 :
			self.roll_controll.reset()
			self.altitude_control.reset()
			self.pitch_control.reset()
			self.yaw_control.reset()
			self.takeoff_time = time.time()
			self.state_altitude = 0
			self.SendLand()
		elif data.data == 3:
			self.roll_controll.reset()
			self.altitude_control.reset()
			self.pitch_control.reset()
			self.yaw_control.reset()
			self.SendEmergency()


	def callback(self, data):
		x = int(-data.x)
		y = int(-data.y)/180.0
		d = -data.z
		state = int(data.w)
		offset = int(640*tan(self.rotX)/(tan(0.52 + self.rotX) + tan(0.52 - self.rotX)))
		if int(data.y) == 0:
			offset = 0
		else:
			offset = int(640*tan(self.rotX)/(tan(0.52 + self.rotX) + tan(0.52 - self.rotX)))

		x = (x + offset)/300.0
		#x = -data.x/300.0
		if state == 2:
			self.roll_controll.setConstants(0.1,0,0)
		

		if ((state == 2) and (self.state_altitude == 0) and (self.status != 2)):
			self.SetCommand(0,0,0,1)
			self.SendCommand()
			time.sleep(5)
			self.SetCommand(0,0,0,0)
			self.SendCommand()
			self.state_altitude = 1

		if x == 0:
			self.roll_controll.last_error = 0
			self.yaw_control.last_error = 0
		if y == 0:
			self.altitude_control.last_error = 0
			self.pitch_control.last_error = 0

		while (time.time() - self.takeoff_time < 8):
			self.roll_controll.reset()

		roll_output = self.roll_controll.update(x)
		altitude_output = self.altitude_control.update(y)
		pitch_output = self.pitch_control.update(y)
		if state == 2:
			yaw_output = self.yaw_control.update(x) 
		elif state == 4:
			yaw_output = self.yaw_control.update(d)

		if state == 0:
			self.SetCommand(roll_output,0,0,altitude_output)
		elif state == 2:
			self.SetCommand(roll_output,pitch_output,0,0)
		else:
			self.SetCommand(roll_output,0.05,yaw_output,0)

		if (time.time() - self.takeoff_time > 8):
			self.SendCommand()
		
		if state == 0:
			self.pub_test.publish(str(roll_output) + "  " + str(altitude_output) + "  " + str(d))
		elif state == 2:
			self.pub_test.publish(str(roll_output) + "  " + str(pitch_output) + "  " + str(d))
		else:
			#self.pub_test.publish(str(yaw_output) + " " + str(d))
			self.pub_test.publish(str(offset) + " " + str(x))



def main(args):
	ic = controller()
	rospy.init_node('controller', anonymous=True)
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)

