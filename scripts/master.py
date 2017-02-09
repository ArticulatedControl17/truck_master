#!/usr/bin/env python
# license removed for brevity
import rospy
from hw_api_ackermann.msg import AckermannDrive
from std_msgs.msg import Bool

class TruckMaster:
    def __init__(self):
        self.go = False
        self.manual = True
        self.pub = rospy.Publisher('truck_cmd', AckermannDrive, queue_size=10)

        rospy.init_node('master', anonymous=False)
        rospy.Subscriber('aut_ackermann_control', AckermannDrive, self.autAckermannHandler)
        rospy.Subscriber('man_ackermann_control', AckermannDrive, self.manualAckermannHandler)
        rospy.Subscriber('manual_control', Bool, self.manualOrAutomaticHandler)
        rospy.Subscriber('dead_mans_grip', Bool, self.goHandler)

    def autAckermannHandler(self,data):
        if not self.manual and self.go:
            self.pub.publish(data)

    def manualAckermannHandler(self,data):
        print "manual", self.manual
        print "go", self.go
        if self.manual and self.go:
            print "publishing data", data
            self.pub.publish(data)

    def manualOrAutomaticHandler(self,data):
        self.manual = data.data

    def goHandler(self,data):
        self.go = data.data
        if not self.go:
            ack = AckermannDrive()
            ack.steering_angle = 0
            ack.speed = 0
            self.pub.publish(ack)

if __name__ == '__main__':
    try:
        tm = TruckMaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
