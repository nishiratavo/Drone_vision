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

class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_W
	PitchBackward    = QtCore.Qt.Key.Key_S
	RollLeft         = QtCore.Qt.Key.Key_A
	RollRight        = QtCore.Qt.Key.Key_D
	YawLeft          = QtCore.Qt.Key.Key_Q
	YawRight         = QtCore.Qt.Key.Key_E
	IncreaseAltitude = QtCore.Qt.Key.Key_Up
	DecreaseAltitude = QtCore.Qt.Key.Key_Down
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	Emergency        = QtCore.Qt.Key.Key_Space


class controller:
	''' Handles the PID control in every axis of the drone deppending on the flight mode '''
	
	def __init__(self):

		self.roll_control = PID(1,0,10)
		self.altitude_control = PID(1,0,0)
		self.pitch_control = PID(0.1,0,0)
		self.yaw_control = PID(1,0,0.1)
		self.command = Twist()

		self.first_time = 0

		self.state_altitude = 0
		self.status = -1
		self.takeoff_time = 0

		self.vy = 0
		self.last_vy = 0
		self.last_time = 0
		self.dy = 0

		self.keyboard_mode = 0

		self.image_pos = rospy.Subscriber("data",Quaternion,self.callback)
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
		self.receive_key = rospy.Subscriber('/takeoff', Int32, self.takeoff_commands) 
		self.keyboard = rospy.Subscriber("/keyboard", Quaternion, self.keyboard_command)
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)
		self.pub_test = rospy.Publisher('/pid_teste',String, queue_size=10)
		self.pub_reset = rospy.Publisher("/position", Int32, queue_size=10)



	def ReceiveNavdata(self,navdata):
		# gets navdata state	
		self.status = navdata.state
		self.rotX = navdata.rotX*(pi/180)
		self.vy = navdata.vx/1000.0



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
		# send the current command
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)


	def takeoff_commands(self,data):
		# send commands and reset the controllers when requested
		if data.data  == 1 :
			self.roll_control.reset()
			self.altitude_control.reset()
			self.pitch_control.reset()
			self.yaw_control.reset()
			self.takeoff_time = time.time()
			self.SendTakeoff()
			self.state_altitude = 0
		elif data.data == 2 :
			self.roll_control.reset()
			self.altitude_control.reset()
			self.pitch_control.reset()
			self.yaw_control.reset()
			self.takeoff_time = time.time()
			#self.state_altitude = 0
			self.SetCommand(0,0,0,0)
			self.SendCommand()
			self.SendLand()
			self.state_altitude = 0
		elif data.data == 3:
			self.roll_control.reset()
			self.altitude_control.reset()
			self.pitch_control.reset()
			self.yaw_control.reset()
			self.SendEmergency()


	def integration(self):
		# integrator for control in the y axis on the front camera mode
		actual_time = time.time()
		dt = actual_time - self.last_time
		delta = (self.vy + self.last_vy)/2
		self.dy += delta*dt
		self.last_time = actual_time
		self.last_vy = self.vy
		return self.dy


	def pointer_follower_front(self, data):
		# controller for the front camera mode
		if self.first_time == 0:
			self.roll_control.setConstants(1,0,2)
			self.altitude_control.setConstants(1,0,0)
			self.pitch_control.setConstants(0.1,0,0.01)
			self.first_time = 1
		x = int(-data.x)/300.0
		y = int(-data.y)/180.0

		if x == 0:
			self.roll_control.last_error = 0
		if y == 0:
			self.pitch_control.last_error = 0

		while (time.time() - self.takeoff_time < 8):
			self.last_time = time.time()

		roll_output = self.roll_control.update(x)
		altitude_output = self.altitude_control.update(y)
		pitch_output = self.pitch_control.update(-self.integration())
		
		self.SetCommand(roll_output,pitch_output,0,altitude_output)

		if (time.time() - self.takeoff_time > 8):
			self.SendCommand()
		
		self.pub_test.publish(str(roll_output) + "  " + str(altitude_output) + " " + str(pitch_output) + " " + str(self.dy) )


	def pointer_follower_bottom(self,data):
		# controller for the bottom camera mode
		if self.first_time == 0:
			self.roll_control.setConstants(0.1,0,0)
			self.pitch_control.setConstants(0.1,0,0)
			self.first_time = 1

		x = int(-data.x)/600.0
		y = int(-data.y)/360.0
		state = int(data.w)


		if ((self.state_altitude == 0) and (self.status != 2)):
			self.SetCommand(0,0,0,0)
			self.SendCommand()
			time.sleep(2)
			self.SetCommand(0,0,0,1)
			self.SendCommand()
			time.sleep(0.5)
			self.SetCommand(0,0,0,0)
			self.SendCommand()
			self.state_altitude = 1

		if x == 0:
			self.roll_control.last_error = 0
		if y == 0:
			self.pitch_control.last_error = 0

		while (time.time() - self.takeoff_time < 8):
			pass

		roll_output = self.roll_control.update(x)
		pitch_output = self.pitch_control.update(y)
		
		self.SetCommand(roll_output,pitch_output,0,0)

		if (time.time() - self.takeoff_time > 8):
			self.SendCommand()
		
		self.pub_test.publish(str(roll_output) + "  " + str(pitch_output))



	def line_follower(self, data):
		# controller for the line follower mode
		if self.first_time == 0:
			self.roll_control.setConstants(1,0,10)
			self.yaw_control.setConstants(1,0,0.1)
			self.first_time = 1

		x = -data.x
		detecting = int(data.y)
		angle = -data.z

		offset = int(640*tan(self.rotX)/(tan(0.52 + self.rotX) + tan(0.52 - self.rotX)))

		if detecting != 0:
			x = (x + offset)/300.0
			pitch_vel = 0.1
		else:
			pitch_vel = 0
			x /= 300.0 

		if x == 0:
			x = self.roll_control.last_error
			self.yaw_control.last_error = 0

		while (time.time() - self.takeoff_time < 8):
			self.roll_control.reset()

		roll_output = self.roll_control.update(x)	
		yaw_output = self.yaw_control.update(angle)

		self.SetCommand(roll_output,pitch_vel,yaw_output,0)

		if (time.time() - self.takeoff_time > 8):
			self.SendCommand()
		
		self.pub_test.publish(str(roll_output) + " " + str(pitch_vel) + " " + str(yaw_output))




	def waypoint(self,data):
		# send commands in the waypoint mode
		x = data.x
		y = data.y
		z = data.z

		while (time.time() - self.takeoff_time < 8):
			self.pub_reset.publish(1)

		#roll_output = self.roll_control.update(x)
		#pitch_output = self.pitch_control.update(y)
		#altitude_output = self.altitude_control.update(z)
		
		self.SetCommand(x,y,0,z)

		if (time.time() - self.takeoff_time > 8):
			self.SendCommand()
		
		self.pub_test.publish(str(x) + "  " + str(y) + "  " + str(z))




	def keyboard_command(self,data):
		if self.keyboard_mode == 1:
			self.SetCommand(data.x,data.y,data.z,data.w)
			self.SendCommand()
			




	def callback(self, data):
		if data.w == 0:
			self.pointer_follower_front(data)
			self.keyboard_mode = 0

		elif data.w == 2:
			self.pointer_follower_bottom(data)
			self.keyboard_mode = 0

		elif data.w == 4:
			self.line_follower(data)
			self.keyboard_mode = 0

		elif data.w == 5:
			self.waypoint(data)
			self.keyboard_mode = 0

		elif data.w == 6:
			self.keyboard_mode = 1

			




def main(args):
	ic = controller()
	rospy.init_node('controller', anonymous=True)
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)

