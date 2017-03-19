#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from errorsmoothie import *
from std_msgs.msg import Bool
from custom_msgs.msg import *
from custom_msgs.srv import *
from error_calc import *
from pid import *
from geometry import *

SWITCH_CAMERA_COOLDOWN = 3

DRIVE_SPEED = 0.51
DRIVE_SPEED_SLOW = 0.43

DRIVE_SPEED_TRAILER = 0.56
DRIVE_SPEED_TRAILER_SLOW = 0.51

SMOOTHING_TIME = 1.0
SMOOTHING_DT = 0.025

LOOKAHEAD = 400

GOAL_LOOKAHEAD =  LOOKAHEAD * 7.0/8

ONLY_FRONT_TAG_LOOKAHEAD = LOOKAHEAD * 0.25
ONLY_FRONT_TAG_TOO_CLOSE_DIST = 2

JOURNEY_START_REQUEST_COOLDOWN = 5
REWORK_POS_UPDATE_COOLDOWN = JOURNEY_START_POS_UPDATE_COOLDOWN = 0.5
SLOWDOWN_DISTANCE = 40

KP = 80
KI = 0.6
KD = 15
WINDUP_GUARD = 100.0


class AutoMaster:
    def __init__(self):
        rospy.init_node('auto_master', anonymous=False)
        
        if rospy.get_param('auto_master/trailer', True):
            self.speed = DRIVE_SPEED_TRAILER
            self.speed_slow = DRIVE_SPEED_TRAILER_SLOW
            print "trailer"
        else:
            self.speed = DRIVE_SPEED
            self.speed_slow = DRIVE_SPEED_SLOW
        
        
        self.latest_point = None
        self.latest_direction = None
        
        self.last_journey_start = 0
        
        self.latest_position_update = 0

        
        
        self.error_calc = ErrorCalc()
        
        self.error_smoothie = ErrorSmoothie(self)
        
        self.pid = PID(KP, KI, KD, WINDUP_GUARD)
        
        self.pub = rospy.Publisher('auto_drive', AckermannDrive, queue_size=10)
        
        self.rework_srv = rospy.Service('rework_path', ReworkPath, self.reworkPathHandler)

        rospy.Subscriber('gv_positions', GulliViewPositions, self.error_smoothie.gvPositionsHandler)
        rospy.Subscriber('position_and_direction', PositionAndDirection, self.positionHandler)
        rospy.Subscriber('path_append', Path, self.pathAppendHandler)
        rospy.Subscriber('dead_mans_switch', Bool, self.deadMansSwitchHandler)
        rospy.Subscriber('start_journey', Bool, self.startJourneyHandler)
        
        print "waiting for journey start cmd"

    
    def updateLatest(self, point = None, direction = None):
        if point != None:
            self.latest_point = point
        if direction != None:
            self.latest_direction = direction
            
        self.latest_position_update = rospy.get_time()
    
    def startJourneyHandler(self,data):
        print "got journey start cmd"
        if data.data:
            
            sj = True
            
            if rospy.get_time() - self.last_journey_start < 5:
                print "chill with the requests bro, last one less than 5 sec ago"
                sj = False
            
            if self.latest_point == None:
                print "no latest point"
                sj = False
            
            if self.latest_direction == None:
                print "no latest direction"
                sj = False
                
            if rospy.get_time() - self.latest_position_update >= JOURNEY_START_POS_UPDATE_COOLDOWN:  #100 sec just for testing
                print "latest position update was ages ago"
                sj = False
            
        
            if sj:
                print "waiting for service..."
                rospy.wait_for_service('request_path')
                print "service available"
                try:
                    pr = rospy.ServiceProxy('request_path', PathRequest)
                    (x,y) = self.latest_point
                    resp1 = pr(Position(x,y), self.latest_direction)
                    if resp1.accepted.data:
                        print "service accepted, starting journey"
                        self.error_calc.reset()
                        self.error_smoothie.reset()
                        self.last_journey_start = rospy.get_time()
                        
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                
                

    def reworkPathHandler(self, data):
        path = data.new_path.path
        self.error_calc.reworkPath(path)
        
        response = ReworkPathResponse()
        
        c = True
        
        if self.latest_point == None:
            print "no latest point"
            c = False
            
        if self.latest_direction == None:
            print "no latest_direction"
            c = False
            
        if rospy.get_time() - self.latest_position_update >= REWORK_POS_UPDATE_COOLDOWN: # 100 just for testing
            print "no recent position update"
            c = False
        
        if c:
            (error, dist) = self.error_calc.calculateError(self.latest_point)
            if dist == 0:
                response.has_passed_last_point = Bool(True)
                response.startposition = Position(self.latest_point[0], self.latest_point[1])
                response.startangle = self.latest_direction
                self.error_calc.reset()
                self.error_smoothie.reset()
                
                ack = AckermannDrive()
                ack.steering_angle = 0
                ack.speed = 0
                self.pub.publish(ack)
                
            else:
                response.has_passed_last_point = Bool(False)
        else:
            response.has_passed_last_point = Bool(False)
            
        return response
            

        

    def positionHandler(self,data):
        p = (data.p.x, data.p.y)
        d = data.direction
        lookAheadPoint = getLookAheadPoint(p,d, LOOKAHEAD)
        
        
        self.updateLatest(p, d)
        
        error, dist = self.error_calc.calculateError(lookAheadPoint)
        
        
        self.processError(error, dist) 
        
        
        
    
    def processError(self, error, dist):
        
        if dist == 0:
            steering_angle_cmd = 0
            speed_cmd = 0
        
        else:
            
            error = error / 1000.0
            steering_angle_cmd = self.pid.update(error)
            
            if dist < SLOWDOWN_DISTANCE:
                
                speed_cmd = self.speed_slow
            else:
                speed_cmd = self.speed
                
        ack = AckermannDrive()
        ack.steering_angle = steering_angle_cmd
        ack.speed = speed_cmd
        self.pub.publish(ack)
            
        
        
        
    
    def pathAppendHandler(self,data):
        print "appending path.."
        self.error_calc.appendPath(data.path)

    def deadMansSwitchHandler(self,data):
        if not data.data:
            self.pid.clear()
    

if __name__ == '__main__':
    am = AutoMaster()
    rospy.spin()
