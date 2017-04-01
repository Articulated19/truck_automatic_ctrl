#!/usr/bin/env python

from math import *
import rospy
from ackermann_msgs.msg import AckermannDrive
from custom_msgs.msg import *
MIN_ANGLE = -16
MAX_ANGLE = 20


# The simulator
class Sim:

    def __init__(self):
        
        

        rospy.init_node('sim', anonymous=False)

        rospy.Subscriber('auto_drive', AckermannDrive, self.ackermannHandler)
        rospy.Subscriber('sim_reset', TruckState, self.resetHandler)

        self.pub = rospy.Publisher('sim_state', TruckState, queue_size=10)

        self.rate = rospy.Rate(50)

        self.latest_sim = rospy.get_time()

        self.lh = 210.0 #270
        self.lt = 490.0 #500

        self.x = 3550.0
        self.y = 5580.0
        self.theta1 = pi/2
        self.theta2 = pi/2

    
    def resetHandler(self, data):
        p = data.p
        self.theta1 = data.theta1
        self.theta2 = data.theta2
        
        self.x = p.x + (self.lh+self.lt)*cos(self.theta1)
        self.y = p.y + (self.lh+self.lt)*sin(self.theta1)
        
        self.latest_sim = rospy.get_time()
    
    def ackermannHandler(self, data):
        
        steering_angle = data.steering_angle
        speed = data.speed

        now = rospy.get_time()
        dt = now - self.latest_sim
        self.latest_sim = now
        #speed = 0
        #steering_angle = 0
        self.processCmd(steering_angle, speed, dt)



    # Reacting to steering command
    def processCmd(self, steering_angle, speed, dt):
        
        
        # Accounting for maximum and minimum steering angle
        if (steering_angle > MAX_ANGLE):
            steering_angle = MAX_ANGLE
        elif (steering_angle < MIN_ANGLE):
            steering_angle = MIN_ANGLE
        
        
        phi = radians(-steering_angle)
        
        speed = speed * 1000    # Converting from m/s to mm/s


        dd = speed * dt

        next_theta1 = self.theta1 + (dd * tan(phi)) / self.lh
        
        next_theta2 = self.theta2 + (dd * sin(self.theta1 - self.theta2)) / self.lt
        
        a1 = self.theta2 - self.theta1
        a2 = next_theta2 - next_theta1
        
        w1 = 0.165
        w2 = 0.06
        w4 = 0.0
        w5 = 0.0
        
        da = a2 - a1
        if a2 > 0:
            if da > 0:
                next_theta2 += da * w4
            else:
                next_theta2 += abs(da) * w5
        else:
            if da < 0:
                next_theta2 -= abs(da) * w4
            else:
                next_theta2 -= da * w5
        
        
        dt1 = next_theta1 - self.theta1
        dt2 = next_theta2 - self.theta2
        alpha = next_theta2 - next_theta1
        
        
        if alpha > 0:
            next_theta2 += (dt2 * w2 + (-dt1) * w1)
        else:
            next_theta2 -= (dt2 * w2 + dt1 * w1)

        
        
        
        next_x = self.x + dd * cos(next_theta1)
        next_y = self.y + dd * sin(next_theta1)

        self.x = next_x
        self.y = next_y
        self.theta1 = next_theta1
        self.theta2 = next_theta2
        
       

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            ts = TruckState(Position(self.x, self.y), self.theta1, self.theta2)
            self.pub.publish(ts)


if __name__ == '__main__':
    s = Sim()
    s.spin()
