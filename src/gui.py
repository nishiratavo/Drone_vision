#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import roslib
import os
roslib.load_manifest('Drone_vision')
import sys
import rospy
from std_msgs.msg import String, Int32, Float32
from drone_status import DroneStatus
from PySide import QtCore, QtGui
from PySide.QtGui import QApplication
from PySide.QtCore import Signal
from PySide.QtGui import  QDesktopWidget
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
import numpy as np
from ai import AttitudeIndicator
import signal
from subprocess import call

class gui(QtGui.QWidget):
	''' Class for the graphical interface '''

	rollPitchSignal = Signal(float, float)
	update_data_signal = Signal(float, float, float, float)
	update_raw_data_signal = Signal(float, float, float, float, float, float)

	def __init__(self):

		super(gui, self).__init__()
		self.mode = rospy.Publisher("/mode", Int32, queue_size = 10)
		self.takeoff = rospy.Publisher("/takeoff", Int32, queue_size = 10)
		self.change_mode = rospy.Publisher("/change_mode", Int32, queue_size = 10)

		vbox = QtGui.QHBoxLayout()
		textbox = QtGui.QVBoxLayout()

		commands = QtGui.QVBoxLayout()

		modes = QtGui.QVBoxLayout()
		mode_label = QtGui.QLabel()
		mode_label.setText("                                       Modes")
		modes.addWidget(mode_label)

		mode_buttons = QtGui.QVBoxLayout()
		self.mode_1 = QtGui.QPushButton("Front Camera")
		self.mode_1.clicked.connect(self.mode_1_clicked)
		mode_buttons.addWidget(self.mode_1)
		self.mode_2 = QtGui.QPushButton("Bottom Camera")
		self.mode_2.clicked.connect(self.mode_2_clicked)
		mode_buttons.addWidget(self.mode_2)
		self.mode_3 = QtGui.QPushButton("Line Follower")
		self.mode_3.clicked.connect(self.mode_3_clicked)
		mode_buttons.addWidget(self.mode_3)
		self.mode_4 = QtGui.QPushButton("Waypoints")
		self.mode_4.clicked.connect(self.mode_4_clicked)
		mode_buttons.addWidget(self.mode_4)

		waypoints_box = QtGui.QHBoxLayout()
		self.waypoints = QtGui.QLineEdit()
		self.waypoints.setText("0,0,0")
		self.waypoints.setFixedWidth(250)
		#waypoints.resize(50,20)
		waypoints_box.addWidget(self.waypoints)

		send = QtGui.QPushButton("Send")
		send.clicked.connect(self.send_clicked)
		send.setFixedWidth(50)
		waypoints_box.addWidget(send)

		blank_space = QtGui.QHBoxLayout()




		buttons = QtGui.QVBoxLayout()
		command_label = QtGui.QLabel()
		command_label.setText("                                     Commands")
		buttons.addWidget(command_label)

		takeoff = QtGui.QPushButton("Take Off")
		takeoff.clicked.connect(self.takeoff_clicked)
		buttons.addWidget(takeoff)
		land = QtGui.QPushButton("Land")
		land.clicked.connect(self.land_clicked)
		buttons.addWidget(land)
		emergency = QtGui.QPushButton("Emergency")
		emergency.clicked.connect(self.emergency_clicked)
		buttons.addWidget(emergency)

		self.save_data = QtGui.QPushButton("Save Data")
		self.save_data.clicked.connect(self.save_data_clicked)
		buttons.addWidget(self.save_data)

		self.attitude_label = QtGui.QLabel()
		self.data1 = QtGui.QLabel()
		self.data2 = QtGui.QLabel()
		self.data3 = QtGui.QLabel()
		self.data4 = QtGui.QLabel()
		#self.frame_attitude = QtGui.QFrame()
		#self.frame_attitude.setFrameStyle(0x0001| 0x0010)

		self.gyro_label = QtGui.QLabel()
		self.gx_label = QtGui.QLabel()
		self.gy_label = QtGui.QLabel()
		self.gz_label = QtGui.QLabel()

		self.accel_label = QtGui.QLabel()
		self.ax_label = QtGui.QLabel()
		self.ay_label = QtGui.QLabel()
		self.az_label = QtGui.QLabel()

		self.info_label = QtGui.QLabel()
		self.battery_label = QtGui.QLabel()

		self.attitude_label.setText("                         Attitude")
		self.data1.setText("Roll")
		self.data2.setText("Pitch")
		self.data3.setText("Yaw")
		self.data4.setText("Altitude")

		self.gyro_label.setText("                         Gyroscope")
		self.gx_label.setText("gx")
		self.gy_label.setText("gy")
		self.gz_label.setText("gz")

		self.accel_label.setText("                         Accelerometer")
		self.ax_label.setText("ax")
		self.ay_label.setText("ay")
		self.az_label.setText("az")

		self.info_label.setText("                         Drone Info")
		self.battery_label.setText("Battery :")

		#self.frame_attitude.addWidget(self.data1)
		#self.frame_attitude.addWidget(self.data2)
		#self.frame_attitude.addWidget(self.data3)
		#self.frame_attitude.addWidget(self.data4)
		#textbox.addWidget(self.frame_attitude)

		textbox.addWidget(self.attitude_label)
		textbox.addWidget(self.data1)
		textbox.addWidget(self.data2)
		textbox.addWidget(self.data3)
		textbox.addWidget(self.data4)

		textbox.addWidget(self.gyro_label)
		textbox.addWidget(self.gx_label)
		textbox.addWidget(self.gy_label)
		textbox.addWidget(self.gz_label)
		textbox.addWidget(self.accel_label)
		textbox.addWidget(self.ax_label)
		textbox.addWidget(self.ay_label)
		textbox.addWidget(self.az_label)

		textbox.addWidget(self.info_label)
		textbox.addWidget(self.battery_label)

		self.wid = AttitudeIndicator()
		modes.addLayout(mode_buttons)
		commands.addLayout(modes)
		commands.addLayout(waypoints_box)
		#commands.addLayout(mode_buttons)
		commands.addLayout(buttons)
		vbox.addLayout(commands)
		vbox.addWidget(self.wid)
		vbox.addLayout(textbox) 
		self.setLayout(vbox)
		self.setGeometry(50, 50, 900, 410)
		self.setWindowTitle('Attitude Indicator')
		self.show()

		self.half_data = -1
		self.data = ["roll", "pitch", "yaw", "altitude", "gx", "gy", "gz", "ax", "ay", "az"]
		self.all_data = []
		#data = self.data
		self.all_data.append(self.data)


		rospy.init_node('gui', anonymous=True)
		self.roll = 0
		self.pitch = 0
		self.rollPitchSignal.connect(self.wid.setRollPitch)
		self.update_data_signal.connect(self.update_data)
		self.update_raw_data_signal.connect(self.update_raw_data)
		self.angle = rospy.Subscriber("ardrone/navdata",Navdata ,self.callback)
		self.raw_data = rospy.Subscriber("ardrone/imu", Imu, self.raw_data_callback)
		self.waypoint_data = rospy.Publisher("/waypoint_receiver", String , queue_size=10)

	def mode_1_clicked(self):
		self.mode.publish(1)
		self.mode_2.setEnabled(False)
		self.mode_3.setEnabled(False)
		self.mode_4.setEnabled(False)

	def mode_2_clicked(self):
		self.mode.publish(2)
		self.mode_1.setEnabled(False)
		self.mode_3.setEnabled(False)
		self.mode_4.setEnabled(False)

	def mode_3_clicked(self):
		self.mode.publish(3)
		self.mode_1.setEnabled(False)
		self.mode_2.setEnabled(False)
		self.mode_4.setEnabled(False)

	def mode_4_clicked(self):
		self.mode.publish(4)
		self.mode_1.setEnabled(False)
		self.mode_2.setEnabled(False)
		self.mode_3.setEnabled(False)


	def send_clicked(self):
		self.waypoint_data.publish(self.waypoints.text())

	def takeoff_clicked(self):
		self.takeoff.publish(1)

	def land_clicked(self):
		self.takeoff.publish(2)
		self.mode_1.setEnabled(True)
		self.mode_2.setEnabled(True)
		self.mode_3.setEnabled(True)
		self.mode_4.setEnabled(True)

	def emergency_clicked(self):
		self.takeoff.publish(3)

	def save_data_clicked(self):
		if self.half_data == -1:
			self.save_data.setText("Stop saving data")
			self.half_data = 0
		else:
			self.save_data.setText("Save Data")
			self.half_data = -1
			path = os.path.join(os.path.expanduser('~'), 'catkin_ws', 'src', 'Drone_vision', 'src', 'data.txt')
			file = open(path, "w")
			file.write('roll pitch yaw altitude gx gy gz ax ay az\n')
			for x in self.all_data:
				line = ' '.join(x)
				file.write(line)
				file.write('\n')
			file.close()
				


	def raw_data_callback(self, data):
		# update graphical interface with the IMU data
		gx = data.angular_velocity.x
		gy = data.angular_velocity.y
		gz = data.angular_velocity.z
		ax = data.linear_acceleration.x
		ay = data.linear_acceleration.y
		az = data.linear_acceleration.z
		self.update_raw_data_signal.emit(gx,gy,gz,ax,ay,az)

		if self.half_data == 1:
			self.data[4] = str(gx)
			self.data[5] = str(gy)
			self.data[6] = str(gz)
			self.data[7] = str(ax)
			self.data[8] = str(ay)
			self.data[9] = str(az)
			self.all_data.append(self.data)
			self.half_data = 0





	def update_raw_data(self,gx,gy,gz,ax,ay,az):
		# formating the data
		gx = str(gx)
		gy = str(gy)
		gz = str(gz)
		ax = str(ax)
		ay = str(ay)
		az = str(az)
		gx = gx[:gx.index(".") + 4]
		gy = gy[:gy.index(".") + 4]
		gz = gz[:gz.index(".") + 4]
		ax = ax[:ax.index(".") + 5]
		ay = ay[:ay.index(".") + 5]
		az = az[:az.index(".") + 5]





		self.gx_label.setText("gx: " + gx + " rad/s")
		self.gy_label.setText("gy: " + gy + " rad/s")
		self.gz_label.setText("gz: " + gz + " rad/s")
		self.ax_label.setText("ax: " + ax + u" m/s²")
		self.ay_label.setText("ay: " + ay + u" m/s²")
		self.az_label.setText("az: " + az + u" m/s²")



	def update_data(self,roll,pitch,yaw,altitude):
		# formating the data

		roll = str(roll)
		pitch = str(pitch)
		yaw = str(yaw)
		altitude = str(altitude)

		roll = roll[:roll.index(".") + 3]
		pitch = pitch[:pitch.index(".") + 3]
		yaw = yaw[:yaw.index(".") + 3]
		altitude = altitude[:altitude.index(".") + 3]


		self.data1.setText("Roll: " + roll + u"°")
		self.data2.setText("Pitch: " + pitch + u"°")
		self.data3.setText("Yaw: " + yaw + u"°")
		self.data4.setText("Altitude: " + altitude + "mm")

	def callback(self, data):
		# called when received data

		roll = data.rotX
		pitch = data.rotY
		yaw = data.rotZ
		altitude = data.altd
		battery = data.batteryPercent
		self.battery_label.setText("Battery: " + str(battery))
		#roll = 20
		#pitch = 30
		self.rollPitchSignal.emit(roll, -pitch)
		self.update_data_signal.emit(roll,pitch,yaw,altitude)

		if self.half_data == 0:
			self.data[0] = str(roll)
			self.data[1] = str(pitch)
			self.data[2] = str(yaw)
			self.data[3] = str(altitude)
			self.half_data = 1
		#self.data1.setText("Roll: " + str(roll))
		#self.data2.setText("Pitch: " + str(pitch))
		#self.data3.setText("Yaw: " + str(yaw))
		#self.data4.setText("Altitude: " + str(altitude))
		#self.wid.setRollPitch(roll,pitch)
		#time.sleep(0.1)

	'''def update_pitch():
		pitch = data.data
		self.wid.setPitch(pitch)'''


def sigint_handler(*args):
    #sys.exit()
    QApplication.quit()
    
def main():
	signal.signal(signal.SIGINT, sigint_handler)  
	QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_X11InitThreads)  
	app = QtGui.QApplication(sys.argv)    
	ex = gui()
	ex.move(50,462)
	
	#rospy.spin()
    
    # Keep checking for sigkill
	#timer = QTimer()
	#timer.start(100) 
	#timer.timeout.connect(lambda: None)  
      
	sys.exit(app.exec_())


if __name__ == '__main__':
	main()
