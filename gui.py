#!/usr/bin/env python

import time
import roslib
roslib.load_manifest('learning')
import sys
import rospy
from std_msgs.msg import String, Int32, Float32
from drone_status import DroneStatus
from PySide import QtCore, QtGui
from PySide.QtGui import QApplication
from PySide.QtCore import Signal
from ardrone_autonomy.msg import Navdata
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
import numpy as np
from ai import AttitudeIndicator
import signal


class gui(QtGui.QWidget):

	rollPitchSignal = Signal(float, float)

	def __init__(self):

		super(gui, self).__init__()
		#self.wid = 0
		#self.init_gui()
		vbox = QtGui.QVBoxLayout()
		self.wid = AttitudeIndicator()
		vbox.addWidget(self.wid)
		hbox = QtGui.QHBoxLayout()
		hbox.addLayout(vbox)
		self.setLayout(hbox)
		self.setGeometry(50, 50, 510, 510)
		self.setWindowTitle('Attitude Indicator')
		self.show()
		rospy.init_node('gui', anonymous=True)
		self.roll = 0
		self.pitch = 0
		self.rollPitchSignal.connect(self.wid.setRollPitch)
		self.angle = rospy.Subscriber("ardrone/navdata",Navdata ,self.callback)

		#self.read_pitch = rospy.Subscriber("ardrone/navdata/rotY", Float32 ,self.update_roll)







	def init_gui(self):

		vbox = QtGui.QVBoxLayout()
		self.wid = AttitudeIndicator()
		vbox.addWidget(self.wid)
		hbox = QtGui.QHBoxLayout()
		hbox.addLayout(vbox)
		self.setLayout(hbox)
		self.setGeometry(50, 50, 510, 510)
		self.setWindowTitle('Attitude Indicator')
		self.show()

	def callback(self, data):

		roll = data.rotX
		pitch = data.rotY
		#roll = 20
		#pitch = 30
		self.rollPitchSignal.emit(roll, pitch)
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
	#rospy.spin()
    
    # Keep checking for sigkill
	#timer = QTimer()
	#timer.start(100) 
	#timer.timeout.connect(lambda: None)  
      
	sys.exit(app.exec_())


if __name__ == '__main__':
	main()



