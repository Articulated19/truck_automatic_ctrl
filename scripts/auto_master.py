#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from errorsmoothie import *
from std_msgs.msg import Bool
from custom_msgs.msg import *
from std_srvs.srv import Trigger
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

JOURNEY_START_REQUEST_COOLDOWN = 15
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
        else:
            self.speed = DRIVE_SPEED
            self.speed_slow = DRIVE_SPEED_SLOW
        
        
        
        self.last_journey_start = 0
        

        
        
        self.error_calc = ErrorCalc()
        
        self.error_smoothie = ErrorSmoothie(self)
        
        self.pid = PID(KP, KI, KD, WINDUP_GUARD)
        
        self.drive_publisher = rospy.Publisher('auto_drive', AckermannDrive, queue_size=10)
        self.position_publisher = rospy.Publisher('truck_state', PositionAndDirection, queue_size=10)
        
        rospy.Subscriber('rework_path', Path, self.reworkPathHandler)

        rospy.Subscriber('gv_positions', GulliViewPositions, self.error_smoothie.gvPositionsHandler)
        rospy.Subscriber('position_and_direction', PositionAndDirection, self.positionHandler)
        rospy.Subscriber('path_append', Path, self.pathAppendHandler)
        rospy.Subscriber('dead_mans_switch', Bool, self.deadMansSwitchHandler)
        rospy.Subscriber('start_journey', Bool, self.startJourneyHandler)
        
        print "waiting for journey start cmd"

    
    def updateLatest(self, point = None, direction = None):
        m = PositionAndDirection()
        
        if point != None:
            x,y = point
            m.p = Position(x,y)
        else:
            m.p = Position(-1,-1)
        
        
        if direction != None:
            m.direction = direction
        else:
            m.direction = -1
        
        self.position_publisher.publish(m)
        
        
    def startJourneyHandler(self,data):
        print "got journey start cmd"
        if data.data:
            
            
            if rospy.get_time() - self.last_journey_start < JOURNEY_START_REQUEST_COOLDOWN:
                print "chill with the requests bro, last one less than 5 sec ago"
            
        
            else:
                print "waiting for service..."
                rospy.wait_for_service('request_path')
                print "service available"
                try:
                    rp = rospy.ServiceProxy('request_path', Trigger)
                    resp = rp()
                    if resp.success.data:
                        print "service accepted, starting journey"
                        self.error_calc.reset()
                        self.error_smoothie.reset()
                        self.last_journey_start = rospy.get_time()
                    else:
                        print resp.message.data
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                
                

    def reworkPathHandler(self, data):
        path = data.new_path.path
        self.error_calc.reworkPath(path)
        
        

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
        self.drive_publisher.publish(ack)
            
        
        
        
    
    def pathAppendHandler(self,data):
        print "appending path.."
        self.error_calc.appendPath(data.path)

    def deadMansSwitchHandler(self,data):
        if not data.data:
            self.pid.clear()
    

if __name__ == '__main__':
    am = AutoMaster()
    rospy.spin()
