#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool
from custom_msgs.msg import *
from custom_msgs.srv import *
from error_calc import *
from pid import *


DRIVE_SPEED = 0.58
DRIVE_SPEED_SLOW = 0.45

SMOOTHING_TIME = 1.0
SMOOTING_DT = 0.025

LOOKAHEAD = 400

KP = 60
KI = 0.6
KD = 15
WINDUP_GUARD = 100.0

class AutoMaster:
    def __init__(self):
        rospy.init_node('auto_master', anonymous=False)
        
        self.latest_point = None
        self.latest_direction = None
        
        self.last_journey_start = rospy.get_time()

		
		self.error_calc = ErrorCalc()
		
		self.error_smoothie = ErrorSmoothie(self.error_calc, SMOOTHING_TIME, SMOOTING_DT, LOOKAHEAD)
		
		self.pid = PID(KP, KI, KD, WINDUP_GUARD)
		
        self.pub = rospy.Publisher(auto_drive, AckermannDrive, queue_size=10)
        
        self.rework_srv = rospy.Service('rework_path', ReworkPath, self.reworkPathHandler)

        rospy.Subscriber('gv_positions', GulliViewPositions, self.gvPositionsHandler)
        rospy.Subscriber('position_and_direction', PositionAndDirection, self.positionHandler)
        rospy.Subscriber('path_append', Path, self.pathAppendHandler)
        rospy.Subscriber('dead_mans_switch', Bool, self.deadMansSwitchHandler)
        rospy.Subscriber('start_journey', Bool, self.startJourneyHandler)

    def startJourneyHandler(self,data):
		if data.data:
			if rospy.get_time() - self.last_journey_start > 5 and self.latest_point != None and self.latest_direction != None:
				
				rospy.wait_for_service('request_path')
				try:
					pr = rospy.ServiceProxy('request_path', PathRequest)
					(x,y) = self.latest_point
					resp1 = pr(Position(x,y), self.latest_direction)
					if resp1.accepted:
						self.error_calc.reset()
						
				except rospy.ServiceException, e:
					print "Service call failed: %s" % e
				
				

    def reworkPathHandler(self, data):
		path = data.path
		self.error_calc.reworkPath(path)
		
		response = ReworkPathResponse()
		
		if self.latest_point != None and self.latest_direction != None:
			(error, dist) = self.error_calc.calculateError(self.latest_point)
			if dist == 0:
				response.has_passed_last_point = True
				response.startposition = Position(self.latest_point[0], self.latest_point[1])
				response.startangle = self.latest_direction
				
				ack = Ackermann()
				ack.steering_angle = 0
				ack.speed = 0
				self.pub.publish(ack)
				
			else:
				response.has_passed_last_point = False
		else:
			response.has_passed_last_point = False
			
		return response
			
				
			
		
		
		

    def gvPositionsHandler(self,data):
        p1 = (data.p1.x, data.p1.y)
        p2 = (data.p2.x, data.p2.y)
        tagid1 = data.tagid1
        tagid2 = data.tagid2
        cameraid = data.cameraid
        
        (error, dist, pub, lookaheadPoint, direction) = self.error_smoothie.makeSmoothie(p1, p2, tagid1, tagid2, cameraid)
        self.processError(error, dist, pub)
        
        if lookaheadPoint != None:
			self.latest_point = lookaheadPoint
			
		if direction != None:
			self.latest_direction = direction
        
        

    def positionHandler(self,data):
		p = (data.p.x, data.p.y)
		d = data.direction
		lookaheadPoint = getLookAheadPoint(p,d, LOOKAHEAD)
		
        (error, dist) = self.error_calc.calculateError(lookaheadPoint)
        self.processError(error, dist, True) 
        
		self.latest_point = lookaheadPoint
		self.latest_direction = d
        
    
    def processError(self, error, dist, pub):
		
		if dist == 0:
			ack = Ackermann()
			ack.steering_angle = 0
			ack.speed = 0
			self.pub.publish(ack)
		
		else:
			
			if pub == False:
				return
			
			error = error / 1000.0
			steering_angle_cmd = self.pid.update(error)
			
			if dist < 50:
				speed_cmd = DRIVE_SPEED_SLOW
			else:
				speed_cmd = DRIVE_SPEED
				
			ack.steering_angle = steering_angle_cmd
			ack.speed = speed_cmd
			self.pub.publish(ack)
			
	    
	    
        
    
    def pathAppendHandler(self,data):
        self.error_calc.appendPath(data.path)

    def deadMansSwitchHandler(self,data):
		if not data.data:
			self.pid.clear()
    

if __name__ == '__main__':
    am = AutoMaster()
    rospy.spin()
