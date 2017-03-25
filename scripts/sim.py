from math import *
MAX_ANGLE = -21
MIN_ANGLE = 16


# The simulator
class Sim:

    def __init__(self):
        


        rospy.init_node('sim', anonymous=False)


        rospy.Subscriber('auto_drive', AckermannDrive, self.ackermannHandler)

        self.pub = rospy.Publisher('sim_state')

        self.latest_sim = rospy.get_time()

        self.lh = 320
        self.lt = 630

        self.x = 1160
        self.y = 7690
        self.theta1 = pi/2
        self.theta2 = pi/2


    def ackermannHandler(self, data):
        steering_angle = data.steering_angle
        speed = data.speed

        rospy.sleep(0.02)

        now = rospy.get_time()
        dt = now - self.latest_sim
        self.latest_sim = now

        x, y, t1, t2 = self.sim.processCmd(steering_angle, speed, dt)

        ts = TruckState(Position(x,y), t1, t2)
        self.sim_publisher.publish(ts)


    # Reacting to steering command
    def processCmd(self, steering_angle, speed, dt):
        
        # Accounting for maximum and minimum steering angle
        if (steering_angle > MAX_ANGLE):
            steering_angle = MAX_ANGLE
        elif (steering_angle < MIN_ANGLE):
            steering_angle = MIN_ANGLE

        phi = radians(steering_angle_deg)
        speed = command.speed * 1000    # Converting from m/s to mm/s


        dd = speed * dt

        next_theta1 = self.theta1 + (dd * tan(phi)) / self.lh
        next_theta2 = self.theta2 + (dd * sin(self.theta1 - self.theta2)) / self.lt
        next_x = self.x + dd * cos(next_theta1)
        next_y = self.y + dd * sin(next_theta1)

        self.x = next_x
        self.y = next_y
        self.theta1 = next_theta
        self.theta2 = next_theta2

        return (self.x, self.y, self.theta1, self.theta2)

if __name__ == '__main__':
    s = Sim()
    rospy.sleep(3)
    s.pub.publish(TruckState(Position(s.x, s.y), s.t1, s.t2))
    s.pub.publish(TruckState(Position(s.x, s.y), s.t1, s.t2))
    rospy.spin()