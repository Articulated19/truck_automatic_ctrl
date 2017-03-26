#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from errorsmoothie import *
from std_msgs.msg import *
from custom_msgs.msg import *
from custom_msgs.srv import *
from error_calc import *
from pid import *
from geometry import *
from math import *

SWITCH_CAMERA_COOLDOWN = 3

DRIVE_SPEED = 0.51
DRIVE_SPEED_SLOW = 0.43

DRIVE_SPEED_TRAILER = 0.51#0.56
DRIVE_SPEED_TRAILER_SLOW = 0.43#0.51

SMOOTHING_TIME = 1.0
SMOOTHING_DT = 0.025

LOOKAHEAD = 500

GOAL_LOOKAHEAD =  LOOKAHEAD * 7.0/8

ONLY_FRONT_TAG_LOOKAHEAD = LOOKAHEAD * 0.25
ONLY_FRONT_TAG_TOO_CLOSE_DIST = 2

JOURNEY_START_REQUEST_COOLDOWN = 15

JOURNEY_START_POS_UPDATE_COOLDOWN = 100
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
        
        self.latest_trailer_angle = None
        self.latest_position = None
        self.latest_theta1 = None
        self.latest_position_update = 0
        self.latest_theta2 = None
        

        self.error_calc = ErrorCalc()
        
        self.error_smoothie = ErrorSmoothie(self)
        
        self.pid = PID(KP, KI, KD, WINDUP_GUARD)
        
        self.drive_publisher = rospy.Publisher('auto_drive', AckermannDrive, queue_size=10)
        self.position_publisher = rospy.Publisher('truck_state', TruckState, queue_size=10)
        
        self.rviz_path_publisher = rospy.Publisher('rviz_path', Path, queue_size=10)
        
        rospy.Subscriber('sim_state', TruckState, self.simStateHandler)
    
        rospy.Subscriber('gv_positions', GulliViewPositions, self.error_smoothie.gvPositionsHandler)
        rospy.Subscriber('dead_mans_switch', Bool, self.deadMansSwitchHandler)
        rospy.Subscriber('trailer_sensor', Float32, self.trailerSensorHandler)

        rospy.Subscriber('path_append', Path, self.pathAppendHandler)
        rospy.Subscriber('goal', Position, self.startJourneyHandler)
        rospy.Subscriber('rework_path', Path, self.reworkPathHandler)
        
        print "waiting for journey start cmd"
        
        
    def trailerSensorHandler(self, data):
        self.updateLatest(trailerAngle = data.data)

    
    def updateLatest(self, point = None, direction = None, trailerAngle = None):
        
        
        if point != None:
            self.latest_position = point
        if direction != None:
            self.latest_theta1 = direction
        if trailerAngle != None:
            self.latest_trailer_angle = trailerAngle
        

        if self.latest_position != None and self.latest_theta1 != None and self.latest_trailer_angle != None:
            m = TruckState()
            m.p = Position(*self.latest_position)
            m.theta1 = self.latest_theta1

            m.theta2 = self.latest_theta2 = radians(self.latest_trailer_angle) + self.latest_direction

            self.latest_position_update = rospy.get_time()
            self.position_publisher.publish(m)
        
        
    def startJourneyHandler(self, data):
        goal = data.x, data.y
        sj = True
        
        if rospy.get_time() - self.last_journey_start < JOURNEY_START_REQUEST_COOLDOWN:
            msg +=  "chill with the requests bro, last one less than 5 sec ago" + "\n"

        if self.latest_theta2 == None:
            msg += "no latest theta2" + "\n"
            sj = False
        
        if self.latest_position == None:
            msg += "no latest point" + "\n"
            sj = False
            
        if self.latest_theta1 == None:
            msg += "no latest direction" + "\n"
            sj = False
            
        if rospy.get_time() - self.latest_position_update >= JOURNEY_START_POS_UPDATE_COOLDOWN:  #100 sec just for testing
            msg += "latest position update was ages ago" + "\n"
            sj = False
        
        if not sj:
            print msg
        else:
            print "waiting for service..."
            rospy.wait_for_service('request_path')
            print "service available"
            try:
                rp = rospy.ServiceProxy('request_path', RequestPath)

                state = TruckState()
                state.p = Position(*self.latest_position)
                state.theta1 = self.latest_theta1
                state.theta2 = self.latest_theta2

                resp = rp(state, Position(*goal)))
                if resp.success:
                    print "service accepted, starting journey"
                    self.error_calc.reset()
                    self.pid.clear()
                    self.error_smoothie.reset()
                    self.last_journey_start = rospy.get_time()
                    self.error_calc.appendPath([state.p])
                else:
                    print resp.message
            except rospy.ServiceException, e:
                print "Service call failed: %s" % e



    def reworkPathHandler(self, data):
        path = data.new_path.path
        self.error_calc.reworkPath(path)

        p = self.error_calc.getPath()
        ms = Path([Position(x,y) for x,y in p])
        self.rviz_path_publisher.publish(ms)
        

        

    def simStateHandler(self,data):
        p = (data.p.x, data.p.y)
        t1 = data.theta1
        t2 = data.theta2

        if t2 == -1:
            t2 = None

        lookAheadPoint = getLookAheadPoint(p, t1, LOOKAHEAD-65)
        
        
        self.updateLatest(p, t1, degrees(t2-t1))
        
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
        print data.path
        self.error_calc.appendPath(data.path)


        p = self.error_calc.getPath()
        ms = Path([Position(x,y) for x,y in p])
        self.rviz_path_publisher.publish(ms)

    def deadMansSwitchHandler(self,data):
        if not data.data:
            self.pid.clear()



    

if __name__ == '__main__':
    am = AutoMaster()
    rospy.spin()
