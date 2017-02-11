#!/usr/bin/env python
import rospy
import sys
from pid_ctrl_pkg.msg import *

class pid:

    def __init__(self):
        print "init"
        self.speed = 0.5
        self.steering_angle = 0
        self.last_time = 0.0
        self.last_error = 0
        self.int = 0

        # Parameters
        self.seta = 0.6
        self.wn = 1.3
        self.delta = 5
        self.lh = 0.21

        self.pub = rospy.Publisher("auto_drive", AckermannDrive, queue_size=10)
        self.sub = rospy.Subscriber("error", Num, self.callback)
        print "waiting for input"

    def callback(self, data):
        self.loop(data)

    def loop(self, data):
        error = data.num

        # Calculating dt
        time = rospy.get_time()
        #time = data.header.stamp.secs + data.header.stamp.nsecs / 1000000000.0
        dt = time - self.last_time
        #if dt > 10:
        #    dt = 0
        self.last_time = time

        # Designing equations
        Ki = self.delta / self.wn
        p = (2 * self.seta * self.wn + self.speed) / (1 - self.delta * self.wn)
        Km = -(self.speed**2) / self.lh
        Kp = (-self.delta * self.wn * p) / Km
        Kd = (self.wn**2 - 2 * self.seta * self.wn * p) / (Kp*Km)

        self.int = self.int + error * dt

        # anti-windup
        if self.int > 0.4 :
            self.int = 0.4
        if self.int < -0.4:
            self.int = -0.4

        dev = (error - self.last_error) / dt
        self.steering_angle = Kp * error + Ki * self.int + Kd * dev

        self.last_error = error

        # Construct the message
        ack = AckermannDrive()
        ack.steering_angle = self.steering_angle
        ack.speed = self.speed

        self.pub.publish(ack)


def main(args):

    pid()
    rospy.init_node('pid', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main(sys.argv)
