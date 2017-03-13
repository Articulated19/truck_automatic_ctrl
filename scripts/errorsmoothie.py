#!/usr/bin/env python
from error_calc import *
import rospy

class ErrorSmoothie:
	def __init__(self, smoothing_time, smoothing_dt):
		self.currentCam = -1
		self.timeoutCam = False
		self.timeoutTime= -1
		self.fromError = 0
		self.errorDifference = 0
		self.lookforToError = False
		self.iteration = 0
		self.updateTime = 0
        
		self.smoothing_time = smoothing_time #1
		self.smoothing_dt = smoothing_dt #0.025
		

	def makeSmoothie(self, error, cameraid):
        
			#look for common camera coverage areas:

		if(self.lookforToError and self.currentCam != cameraid):
			#calculte camera adjumstment difference
			self.errorDifference = (self.fromError - error)/ (self.smoothing_time / float(self.smoothing_dt))
			self.lookforToError=False

		if(cameraid != self.currentCam and self.timeoutCam==False):
			#found new camera
			self.currentCam = cameraid
			self.timeoutCam = True
			self.timeoutTime = rospy.get_time()+ rospy.Duration(self.smoothing_time, 0).to_sec()
			self.updateTime = rospy.get_time()+ rospy.Duration(self.smoothing_dt, 0).to_sec()
			self.iteration = 1
			self.fromError = error
			self.lookforToError = True
		if(rospy.get_time() > self.timeoutTime):
			#three seconds pass, should not  be in area where 2 cameras look
			timeoutCam = False
		if(cameraid == self.currentCam):
			if(self.timeoutCam and rospy.get_time() > self.updateTime):
				#calculate camera adjustment difference one time step
				self.updateTime = rospy.get_time()+ rospy.Duration(self.smoothing_dt, 0).to_sec()
				self.iteration= self.iteration+1

			# publishing only one camera
			error = error  -self.errorDifference*self.iteration
			#rospy.loginfo("error is: %s", error)
			#rospy.loginfo(rospy.get_caller_id() + "I heard x1: %s and y1: %s, and cameraid1: %s, tagid1: %s" , msg.x1, msg.y1,cameraid, msg.tagid1)
			pub = True
		else:
			pub = False
		return (error, pub)
