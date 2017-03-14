#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from errorsmoothie import *
from std_msgs.msg import Bool
from custom_msgs.msg import *
from custom_msgs.srv import *
from error_calc import *
from pid import *


DRIVE_SPEED = 0.58
DRIVE_SPEED_SLOW = 0.45

SMOOTHING_TIME = 1.0
SMOOTING_DT = 0.025

LOOKAHEAD = 2

KP = 60
KI = 0.6
KD = 15
WINDUP_GUARD = 100.0


class AutoMaster:
    def __init__(self):
        rospy.init_node('auto_master', anonymous=False)
        
        self.latest_point = None
        self.latest_direction = None
        
        self.last_journey_start = 0
        
        self.latest_position_update = 0

        
        self.error_calc = ErrorCalc()
        
        self.error_smoothie = ErrorSmoothie(SMOOTHING_TIME, SMOOTING_DT)
        self.prev_error = 0
        self.prev_dist = 0
        
        self.pid = PID(KP, KI, KD, WINDUP_GUARD)
        
        self.pub = rospy.Publisher('auto_drive', AckermannDrive, queue_size=10)
        
        self.rework_srv = rospy.Service('rework_path', ReworkPath, self.reworkPathHandler)

        rospy.Subscriber('gv_positions', GulliViewPositions, self.gvPositionsHandler)
        rospy.Subscriber('position_and_direction', PositionAndDirection, self.positionHandler)
        rospy.Subscriber('path_append', Path, self.pathAppendHandler)
        rospy.Subscriber('dead_mans_switch', Bool, self.deadMansSwitchHandler)
        rospy.Subscriber('start_journey', Bool, self.startJourneyHandler)

    def startJourneyHandler(self,data):
        print "start journey request"
        if data.data:
            if rospy.get_time() - self.last_journey_start > 5 and self.latest_point != None and self.latest_direction != None and rospy.get_time() - self.latest_position_update < 100:  #100 sec just for testing
                print "waiting for service..."
                rospy.wait_for_service('request_path')
                print "service available"
                try:
                    pr = rospy.ServiceProxy('request_path', PathRequest)
                    (x,y) = self.latest_point
                    resp1 = pr(Position(x,y), self.latest_direction)
                    if resp1.accepted.data:
                        print "accepted"
                        self.error_calc.reset()
                        
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                
                

    def reworkPathHandler(self, data):
        path = data.new_path.path
        self.error_calc.reworkPath(path)
        
        response = ReworkPathResponse()
        
        if self.latest_point == None:
            print "no latest point"
        if self.latest_direction == None:
            print "no latest_direction"
        if rospy.get_time() - self.latest_position_update >= 100:
            print "no recent position update"
        
        if self.latest_point != None and self.latest_direction != None and rospy.get_time() - self.latest_position_update < 100: #100 just for testing
            (error, dist) = self.error_calc.calculateError(self.latest_point)
            if dist == 0:
                response.has_passed_last_point = Bool(True)
                response.startposition = Position(self.latest_point[0], self.latest_point[1])
                response.startangle = self.latest_direction
                self.error_calc.reset()
                
                ack = AckermannDrive()
                ack.steering_angle = 0
                ack.speed = 0
                self.pub.publish(ack)
                
            else:
                response.has_passed_last_point = Bool(False)
        else:
            response.has_passed_last_point = Bool(False)
            
        return response
            
                
            
        
        
        

    def gvPositionsHandler(self,data):
        p1 = (data.p1.x, data.p1.y)
        p2 = (data.p2.x, data.p2.y)
        tagid1 = data.tagid1
        tagid2 = data.tagid2
        cameraid = data.cameraid
        
        if (p2 == (0,0) and tagid2==0):
            print "ONE MESSAGE ZERO*************"
            #only one tag out
            error = self.prev_error
            dist = self.prev_dist
            
            self.latest_point = None
            self.latest_direction = None
            
        else:
            #two tags
            #front == id 2
            #back == id 1
            if tagid1 !=1:
                tp = p1
                p1 = p2
                p2 = tp
                
            
            direction = getDirection(p1,p2)
            lookAheadPoint = getLookAheadPoint(p2, direction, LOOKAHEAD)
            error,dist = self.error_calc.calculateError(lookAheadPoint)
            
            self.latest_point = p2
            self.latest_direction = direction
            self.latest_position_update = rospy.get_time()
            self.prevError = error
            self.prevDist = dist
        
        print "lookaheadpoint", lookAheadPoint
        print "direction", direction
        print "error", error
        print "dist", dist
        
        smooth_error, pub = self.error_smoothie.makeSmoothie(error, cameraid)
        
        
        print "smooth_error", smooth_error
        print "pub", pub
        
        self.processError(smooth_error, dist, pub)
        
        
        

    def positionHandler(self,data):
        print "got position", data
        p = (data.p.x, data.p.y)
        d = data.direction
        lookAheadPoint = getLookAheadPoint(p,d, LOOKAHEAD)
        
        
        self.latest_point = p
        self.latest_direction = d
        self.latest_position_update = rospy.get_time()
        
        error, dist = self.error_calc.calculateError(lookAheadPoint)
        
        
        
        print "lookaheadpoint", lookAheadPoint
        print "error", error
        print "dist", dist
        
        self.processError(error, dist, True) 
        
        
        
    
    def processError(self, error, dist, pub):
        
        if dist == 0:
            ack = AckermannDrive()
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
                
            ack = AckermannDrive()
            ack.steering_angle = steering_angle_cmd
            ack.speed = speed_cmd
            self.pub.publish(ack)
            
        
        
        
    
    def pathAppendHandler(self,data):
        print "path appended"
        self.error_calc.appendPath(data.path)

    def deadMansSwitchHandler(self,data):
        if not data.data:
            self.pid.clear()
    

if __name__ == '__main__':
    am = AutoMaster()
    rospy.spin()
