import time

class PID:
	
	def __init__(self, kp, ki, kd):
		self.last_error = 0
		self.integral = 0
		self.last_run = time.time()
		self.kp = kp
		self.ki = ki
		self.kd = kd

	def update(self, error):
		self.integral += self.ki * float(self.last_error + error)/2
		now = time.time()
		derivative = self.kd * float(error - self.last_error)/ (now - self.last_run)
		self.last_error = error
		self.last_run = now
		output = self.integral + derivative + self.kp * error
		output = min(1, max(-1, output))
		return output


	def setConstants(self, kp, ki, kd):
		self.last_error = 0
		self.integral = 0
		self.last_run = time.time()
		self.kp = kp
		self.ki = ki
		self.kd = kd


	def reset(self):
		self.last_error = 0
		self.integral = 0
		self.last_run = time.time()