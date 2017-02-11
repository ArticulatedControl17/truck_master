#!/usr/bin/env python
import rospy
from hw_api_ackermann.msg import AckermannDrive
from std_msgs.msg import Bool

class TruckMaster:
    def __init__(self):
        self.go = False
        self.manual = True
        self.pub = rospy.Publisher('master_drive', AckermannDrive, queue_size=10)

        rospy.init_node('master', anonymous=False)
        rospy.Subscriber('auto_drive', AckermannDrive, self.autAckermannHandler)
        rospy.Subscriber('man_drive', AckermannDrive, self.manualAckermannHandler)
        rospy.Subscriber('auto_ctrl', Bool, self.manualOrAutomaticHandler)
        rospy.Subscriber('dead_mans_switch', Bool, self.goHandler)

    def autAckermannHandler(self,data):
        if not self.manual and self.go:
            self.pub.publish(data)

    def manualAckermannHandler(self,data):
        if self.manual and self.go:
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
    tm = TruckMaster()
    rospy.spin()
