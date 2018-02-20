#!/usr/bin/env python
"""
Copyright (c) 2017, Lars Niklasson
Copyright (c) 2017, Filip Slottner Seholm
Copyright (c) 2017, Fanny Sandblom
Copyright (c) 2017, Kevin Hoogendijk
Copyright (c) 2017, Nils Andren
Copyright (c) 2017, Alicia Gil Martin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
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
from geometry_msgs.msg import PoseWithCovarianceStamped

SWITCH_CAMERA_COOLDOWN = 3

DRIVE_SPEED = 0.49
DRIVE_SPEED_SLOW = 0.47

DRIVE_SPEED_SIM = 0.45
DRIVE_SPEED_SIM_SLOW = 0.45

SMOOTHING_TIME = 1.0
SMOOTHING_DT = 0.025

#great pid for truck in lab: la = 350 + 100 , kp= 100, ki = 0.6, kd = 15

LOOKAHEAD = 300
LOOKAHEAD_SIM = 250

GOAL_LOOKAHEAD =  LOOKAHEAD * 7.0/8

ONLY_FRONT_TAG_LOOKAHEAD = LOOKAHEAD * 0.25
ONLY_FRONT_TAG_TOO_CLOSE_DIST = 2

JOURNEY_START_REQUEST_COOLDOWN = 15

JOURNEY_START_POS_UPDATE_COOLDOWN = 10
SLOWDOWN_DISTANCE = 40

MAX_TRAILER_ANGLE = 40.0
SPEED_INCREASE_COEFFICIENT = 0.2


KP = 100
KI = 0.6
KD = 15

KP_SIM = 200
KI_SIM = 0
KD_SIM = 15


WINDUP_GUARD = 100.0


class AutoMaster:
    def __init__(self):
        rospy.init_node('auto_master', anonymous=False)

        self.sim = rospy.get_param('auto_master/sim')

        if self.sim:
            self.speed = DRIVE_SPEED_SIM
            self.speed_slow = DRIVE_SPEED_SIM_SLOW
            self.kp = KP_SIM
            self.ki = KI_SIM
            self.kd = KD_SIM
            self.la_dist = LOOKAHEAD_SIM
        else:
            self.speed = DRIVE_SPEED
            self.speed_slow = DRIVE_SPEED_SLOW
            self.kp = KP
            self.ki = KI
            self.kd = KD
            self.la_dist = LOOKAHEAD

        self.last_journey_start = 0

        self.latest_trailer_angle = None
        self.latest_position = None
        self.latest_theta1 = None
        self.latest_position_update = 0
        self.latest_theta2 = None

        self.lock_stop = False


        self.error_calc = ErrorCalc()

        self.error_smoothie = ErrorSmoothie(self)

        self.pid = PID(self.kp, self.ki, self.kd, WINDUP_GUARD)

        self.drive_publisher = rospy.Publisher('auto_drive', AckermannDrive, queue_size=10)
        self.position_publisher = rospy.Publisher('truck_state', TruckState, queue_size=10)

        self.rviz_path_publisher = rospy.Publisher('rviz_path', Path, queue_size=10)

        rospy.Subscriber('sim_state', TruckState, self.simStateHandler)

        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initPoseCallback)
        rospy.Subscriber('gv_positions', GulliViewPositions, self.error_smoothie.gvPositionsHandler)
        rospy.Subscriber('dead_mans_switch', Bool, self.deadMansSwitchHandler)
        rospy.Subscriber('trailer_sensor', Float32, self.trailerSensorHandler)

        rospy.Subscriber('path_append', Path, self.pathAppendHandler)
        rospy.Subscriber('truck_goals', Path, self.startJourneyHandler)
        rospy.Subscriber('path_rework', Path, self.reworkPathHandler)

        rospy.Subscriber('section_lock', String, self.sectionLockHandler)

        print "waiting for journey start cmd"

    def sectionLockHandler(self, data):
        if data.data == 'stop':
            self.lock_stop = True
        else:
            self.lock_stop = False

    def initPoseCallback(self, data):
        self.error_calc.reset()
        self.pid.clear()
        self.error_smoothie.reset()

    def trailerSensorHandler(self, data):
        self.updateLatest(trailerAngle = data.data)


    def updateLatest(self, point = None, direction = None, trailerAngle = None):


        if point != None:
            self.latest_position = point
        if direction != None:
            self.latest_theta1 = direction
        if trailerAngle != None:
            self.latest_trailer_angle = trailerAngle


        if self.latest_position != None and self.latest_theta1 != None:

            if self.latest_trailer_angle == None:
                return

            m = TruckState()
            m.p = Position(*self.latest_position)
            m.theta1 = self.latest_theta1

            m.theta2 = self.latest_theta2 = radians(self.latest_trailer_angle) + self.latest_theta1

            self.latest_position_update = rospy.get_time()
            self.position_publisher.publish(m)


    def startJourneyHandler(self, data):
        goals = data
        sj = True

        msg = ""

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

                resp = rp(state, goals)
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
        print "reworked path"
        path = data.path
        self.error_calc.reworkPath(path)


        pa = self.error_calc.getPath()
        ms = Path([Position(p.x,p.y) for p in pa])
        self.rviz_path_publisher.publish(ms)




    def simStateHandler(self,data):
        
        p = (data.p.x, data.p.y)
        t1 = data.theta1
        t2 = data.theta2

        if t2 == -1:
            t2 = None

        lookAheadPoint = getLookAheadPoint(p, t1, self.la_dist-65)

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

            if not self.sim:
                speed_cmd += abs(self.latest_trailer_angle) * SPEED_INCREASE_COEFFICIENT / MAX_TRAILER_ANGLE

        ack = AckermannDrive()
        ack.steering_angle = steering_angle_cmd
        ack.speed = speed_cmd

        if self.lock_stop:
            ack.speed = 0

        self.drive_publisher.publish(ack)





    def pathAppendHandler(self,data):
        print "appending path.."
        print data.path
        print "path before", self.error_calc.getPath()
        self.error_calc.appendPath(data.path)

        pa = self.error_calc.getPath()

        print "path after", pa
        ms = Path([Position(p.x,p.y) for p in pa])
        self.rviz_path_publisher.publish(ms)

    def deadMansSwitchHandler(self,data):
        if not data.data:
            self.pid.clear()





if __name__ == '__main__':
    am = AutoMaster()
    rospy.spin()
