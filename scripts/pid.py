#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int64


PUBLISH_TOPIC = "auto_drive"
SUB_TOPIC = "error"

#MAX_STEERING = ?
#MIN_STEERING = ?
#ZERO_STEERING = ?


class Pid:

    def __init__(self):
        rospy.loginfo("init")

        self.speed = 0.8
        self.steering_angle = 0
        self.last_time = 0.0
        self.last_error = 0
        self.int = 0

        # Parameters
        self.seta = 0.5
        self.wn = 5
        self.delta = 5
        self.lh = 0.21

        self.pub = rospy.Publisher(PUBLISH_TOPIC, AckermannDrive, queue_size=10)
        self.sub = rospy.Subscriber(SUB_TOPIC, Int64, self.callback)
        
        rospy.loginfo("waiting for input from topic 'error'")

    def callback(self, data):
        self.loop(data)

    def loop(self, data):
        error = data.data
        rospy.loginfo("received error: %i", error)

        # Calculating dt
        time = rospy.get_time()
        #time = data.header.stamp.secs + data.header.stamp.nsecs / 1000000000.0
        dt = time - self.last_time
        #if dt > 10:
        #    dt = 0
        self.last_time = time

        # Designing equations
        Ki = (self.delta / self.wn) * 0
        p = (self.wn**2 + self.speed) / (2 * self.seta * self.wn - self.delta * self.wn)
        Km = -(self.speed**2) / self.lh
        Kp = (-self.delta * self.wn * p) / Km
        Kd = ((2 * self.delta * self.wn - p) / (Kp * Km)) * 0

        self.int = self.int + error * dt

        # anti-windup
        #if self.int > 0.4 :
        #    self.int = 0.4
        #if self.int < -0.4:
        #    self.int = -0.4

        dev = (error - self.last_error) / dt
        self.steering_angle = Kp * error + Ki * self.int + Kd * dev
        #self.steering_angle = Kp * error + (1 / Ki * self.int - Kd * dev) * Kp

        self.last_error = error

        # Construct the message
        ack = AckermannDrive()
        ack.steering_angle = self.steering_angle
        ack.speed = self.speed

        self.pub.publish(ack)
        rospy.loginfo("published: [angle: %f, speed: %f]", ack.steering_angle, ack.speed)


def main(args):
    rospy.init_node('pid', anonymous=True)
    Pid()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
 
if __name__ == '__main__':
    main(sys.argv)


# To manually publish messages on 'error' topic: $ rostopic pub /error std_msgs/Int64 -- [value]
