#!/usr/bin/env python
from error_calc import *
import rospy
from math import atan

def getDirection((x1,y1), (x2,y2)):
	if x2 == x1 return None
	return atan((y2-y1)/(x2-x1))

class ErrorSmoothie:
	def __init__(self, _error_calc, smoothing_time, smoothing_dt, lookahead):
		self.ec = _error_calc
		self.currentCam = -1
		self.timeoutCam = False
		self.timeoutTime= -1
		self.fromError = 0
		self.errorDifference = 0
		self.lookforToError = False
		self.iteration = 0
		self.updateTime = 0
		self.prevError = 0
		self.prevDist
		
		self.lookahead = lookahead
		self.smoothing_time = smoothing_time #1
		self.smoothing_dt = smoothing_dt #0.025
		

	def makeSmoothie(self, p1, p2, tagid1, tagid2, cameraid):
		if (p2 == (0,0) and tagid2==0):
			print "ONE MESSAGE ZERO*************"
			#only one tag out
			error = self.prevError
			dist = self.prevDist
			lookaheadPoint = None
			direction = None
		else:
			#two tags
			#front == id 2
			#back == id 1
			if tagid1 ==1:
				#nr 1 is back and nr 2 is front
				lookAheadPoint = getLookAheadPoint(p1,p2,self.lookahead)
				direction = getDirection(p1,p2)
				error,dist = ec.calculateError(lookAheadPoint)
				
			else:
				#nr 2 is back and nr 1 is front
				
				lookAheadPoint = getLookAheadPoint(p2,p1,self.lookahead)
				direction = getDirection(p2,p1)
				error,dist = self.ec.calculateError(lookAheadPoint)
				
			
			if dist == 0:
				return (-1,0,False, lookaheadPoint)
			self.prevError = error
			self.prevDist = dist
			
		
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
		return (error, dist, pub, lookaheadPoint, direction)
