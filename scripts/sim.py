#!/usr/bin/env python

from math import *
import rospy
from ackermann_msgs.msg import AckermannDrive
from custom_msgs.msg import *
MIN_ANGLE = -16
MAX_ANGLE = 19


# The simulator
class Sim:

    def __init__(self):
        
        

        rospy.init_node('sim', anonymous=False)

        rospy.Subscriber('auto_drive', AckermannDrive, self.ackermannHandler)
        rospy.Subscriber('sim_reset', TruckState, self.resetHandler)

        self.pub = rospy.Publisher('sim_state', TruckState, queue_size=10)

        self.rate = rospy.Rate(50)

        self.latest_sim = rospy.get_time()

        self.lh = 270.0 #270
        self.lt = 445 + 102.5/2 + 50 #500


        self.x = 3550.0
        self.y = 5580.0
        self.theta1 = pi/2
        self.theta2 = pi/2

    
    def resetHandler(self, data):
        p = data.p
        self.theta1 = data.theta1
        self.theta2 = data.theta2
        
        
        self.x = p.x + (135 + self.lh+self.lt)*cos(self.theta1)
        self.y = p.y + (135 + self.lh+self.lt)*sin(self.theta1)
        
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

        
        dt1 = (dd * tan(phi)) / self.lh
        next_theta1 = self.theta1 + dt1
        
        r = 50.0
        x = sqrt(dd*dd + (r*dt1)**2)
        
        print x-dd
        
        #nt1
        
        t1_avg = (next_theta1 + self.theta1)/2
        
        next_theta2 = self.theta2 + (x * (sin(atan2(r*dt1, dd) + next_theta1 - self.theta2))) / self.lt
        
        

        
        dx = self.x - self.lh * cos(self.theta1)
        dy = self.y - self.lh * sin(self.theta1)
        
        
        next_x = dx + dd * cos(next_theta1) + self.lh * cos(next_theta1)
        next_y = dy + dd * sin(next_theta1) + self.lh * sin(next_theta1)


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
