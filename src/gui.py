#!/usr/bin/env python

import time
import roslib
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


class gui(QtGui.QWidget):

	rollPitchSignal = Signal(float, float)
	update_data_signal = Signal(float, float, float, float)
	update_raw_data_signal = Signal(float, float, float, float, float, float)

	def __init__(self):

		super(gui, self).__init__()
		#self.wid = 0
		#self.init_gui()
		'''vbox = QtGui.QVBoxLayout()
		self.wid = AttitudeIndicator()
		vbox.addWidget(self.wid)
		hbox = QtGui.QHBoxLayout()
		hbox.addLayout(vbox)
		self.setLayout(hbox)
		self.setGeometry(50, 50, 510, 510)
		self.setWindowTitle('Attitude Indicator')
		self.show()'''
		vbox = QtGui.QHBoxLayout()
		textbox = QtGui.QVBoxLayout()

		commands = QtGui.QVBoxLayout()

		modes = QtGui.QVBoxLayout()
		mode_label = QtGui.QLabel()
		mode_label.setText("                                       Modes")
		modes.addWidget(mode_label)

		mode_buttons = QtGui.QVBoxLayout()
		mode_1 = QtGui.QPushButton("Front Camera")
		mode_buttons.addWidget(mode_1)
		mode_2 = QtGui.QPushButton("Bottom Camera")
		mode_buttons.addWidget(mode_2)
		mode_3 = QtGui.QPushButton("Line Follower")
		mode_buttons.addWidget(mode_3)
		mode_4 = QtGui.QPushButton("Waypoints")
		mode_buttons.addWidget(mode_4)

		waypoints_box = QtGui.QHBoxLayout()
		#waypoints = QtGui.QLineEdit()
		#waypoints.setText("0,0,0")
		#waypoints.resize(50,20)
		#waypoints_box.addWidget(waypoints)

		send = QtGui.QPushButton("Send")
		waypoints_box.addWidget(send)

		blank_space = QtGui.QHBoxLayout()




		buttons = QtGui.QVBoxLayout()
		command_label = QtGui.QLabel()
		command_label.setText("                                     Commands")
		buttons.addWidget(command_label)

		takeoff = QtGui.QPushButton("Take Off")
		buttons.addWidget(takeoff)
		land = QtGui.QPushButton("Land")
		buttons.addWidget(land)
		emergency = QtGui.QPushButton("Emergency")
		buttons.addWidget(emergency)
		finish = QtGui.QPushButton("Finish")
		buttons.addWidget(finish)


		self.data1 = QtGui.QLabel()
		self.data2 = QtGui.QLabel()
		self.data3 = QtGui.QLabel()
		self.data4 = QtGui.QLabel()
		self.gx_label = QtGui.QLabel()
		self.gy_label = QtGui.QLabel()
		self.gz_label = QtGui.QLabel()
		self.ax_label = QtGui.QLabel()
		self.ay_label = QtGui.QLabel()
		self.az_label = QtGui.QLabel()
		self.data1.setText("Roll")
		self.data2.setText("Pitch")
		self.data3.setText("Yaw")
		self.data4.setText("Altitude")
		self.gx_label.setText("gx")
		self.gy_label.setText("gy")
		self.gz_label.setText("gz")
		self.ax_label.setText("ax")
		self.ay_label.setText("ay")
		self.az_label.setText("az")
		textbox.addWidget(self.data1)
		textbox.addWidget(self.data2)
		textbox.addWidget(self.data3)
		textbox.addWidget(self.data4)
		textbox.addWidget(self.gx_label)
		textbox.addWidget(self.gy_label)
		textbox.addWidget(self.gz_label)
		textbox.addWidget(self.ax_label)
		textbox.addWidget(self.ay_label)
		textbox.addWidget(self.az_label)
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


		rospy.init_node('gui', anonymous=True)
		self.roll = 0
		self.pitch = 0
		self.rollPitchSignal.connect(self.wid.setRollPitch)
		self.update_data_signal.connect(self.update_data)
		self.update_raw_data_signal.connect(self.update_raw_data)
		self.angle = rospy.Subscriber("ardrone/navdata",Navdata ,self.callback)
		self.raw_data = rospy.Subscriber("ardrone/imu", Imu, self.raw_data_callback)

		#self.read_pitch = rospy.Subscriber("ardrone/navdata/rotY", Float32 ,self.update_roll)



	def init_gui(self):

		vbox = QtGui.QHBoxLayout()
		textbox = QtGui.QVBoxLayout()
		buttons = QtGui.QVBoxLayout()
		button = QtGui.QLabel()
		data1 = QtGui.QLabel()
		data2 = QtGui.QLabel()
		data3 = QtGui.QLabel()
		data4 = QtGui.QLabel()
		button.setText("teste")
		data1.setText("Roll")
		data2.setText("Pitch")
		data3.setText("Yaw")
		data4.setText("Altitude")
		buttons.addWidget(button)
		textbox.addWidget(data1)
		textbox.addWidget(data2)
		textbox.addWidget(data3)
		textbox.addWidget(data4)

		self.wid = AttitudeIndicator()

		vbox.addWidget(self.wid)

		vbox.addLayout(textbox) 
		vbox.addLayout(buttons)
		self.setLayout(vbox)
		self.setGeometry(50, 50, 710, 510)
		self.setWindowTitle('Attitude Indicator')
		self.show()









		'''vbox = QtGui.QVBoxLayout()
		self.wid = AttitudeIndicator()
		vbox.addWidget(self.wid)
		hbox = QtGui.QHBoxLayout()
		hbox.addLayout(vbox)
		self.setLayout(hbox)
		self.setGeometry(50, 50, 510, 510)
		self.setWindowTitle('Attitude Indicator')
		self.show()'''


	def raw_data_callback(self, data):
		gx = data.angular_velocity.x
		gy = data.angular_velocity.y
		gz = data.angular_velocity.z
		ax = data.linear_acceleration.x
		ay = data.linear_acceleration.y
		az = data.linear_acceleration.z
		self.update_raw_data_signal.emit(gx,gy,gz,ax,ay,az)

	def update_raw_data(self,gx,gy,gz,ax,ay,az):
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





		self.gx_label.setText("gx: " + gx)
		self.gy_label.setText("gy: " + gy)
		self.gz_label.setText("gz: " + gz)
		self.ax_label.setText("ax: " + ax)
		self.ay_label.setText("ay: " + ay)
		self.az_label.setText("az: " + az)



	def update_data(self,roll,pitch,yaw,altitude):

		roll = str(roll)
		pitch = str(pitch)
		yaw = str(yaw)
		altitude = str(altitude)

		roll = roll[:roll.index(".") + 3]
		pitch = pitch[:pitch.index(".") + 3]
		yaw = yaw[:yaw.index(".") + 3]
		altitude = altitude[:altitude.index(".") + 3]


		self.data1.setText("Roll: " + roll)
		self.data2.setText("Pitch: " + pitch)
		self.data3.setText("Yaw: " + yaw)
		self.data4.setText("Altitude: " + altitude)

	def callback(self, data):

		roll = data.rotX
		pitch = data.rotY
		yaw = data.rotZ
		altitude = data.altd
		#roll = 20
		#pitch = 30
		self.rollPitchSignal.emit(roll, -pitch)
		self.update_data_signal.emit(roll,pitch,yaw,altitude)
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
