#!/usr/bin/env python
# license removed for brevity
import rospy
#import sys
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Int64
import pid
PUBLISH_TOPIC = 'auto_drive'
SUB_TOPIC = 'error'
DMS_TOPIC = "dead_mans_switch"

class PIDNode:

    def __init__(self):

        self.speed = rospy.get_param("pid/speed", 0.4)
        self.Kp = rospy.get_param("pid/Kp", 2)
        self.Ki = rospy.get_param("pid/Ki", 0)
        self.Kd = rospy.get_param("pid/Kd", 0)

        self.pid = Pid(Kp, Ki, Kd)


        rospy.init_node('pid', anonymous=True)

        self.pub = rospy.Publisher(PUBLISH_TOPIC, AckermannDrive, queue_size=10)
        self.sub = rospy.Subscriber(SUB_TOPIC, Int64, self.callback)
        self.dms_sub = rospy.Subscriber(DMS_TOPIC, Bool, self.dmsCallback)

        rospy.loginfo("waiting for input from topic '%s'", SUB_TOPIC)
        

    def callback(self, data):

        error = data.data / 1000.0
        steering_angle = self.pid.update(error)


        # Construct the message
        ack = AckermannDrive()
        ack.steering_angle = steering_angle
        ack.speed = this.speed

        self.pub.publish(ack)

    def dmsCallback(self, data):
        dms = data.data
        if not dms:
            pid.clear()


if __name__ == '__main__':
    
    PIDNode()
    rospy.spin()


# To manually publish messages on 'error' topic: $ rostopic pub /error std_msgs/Int64 -- [value]
