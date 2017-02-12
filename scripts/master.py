#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool

PUBLISH_TOPIC = 'master_drive'

class TruckMaster:
    def __init__(self):
        self.dead_mans_switch = False
        self.auto_ctrl = False
        self.pub = rospy.Publisher(PUBLISH_TOPIC, AckermannDrive, queue_size=10)

        self.last_dms_msg = -1

        rospy.init_node('master', anonymous=False)
        rospy.Subscriber('auto_drive', AckermannDrive, self.autoDriveHandler)
        rospy.Subscriber('man_drive', AckermannDrive, self.manualDriveHandler)
        rospy.Subscriber('auto_ctrl', Bool, self.autoCtrlHandler)
        rospy.Subscriber('dead_mans_switch', Bool, self.deadMansSwitchHandler)

        rospy.loginfo('Init done, publishes to /%s', PUBLISH_TOPIC)

    def autoDriveHandler(self,data):
        if self.auto_ctrl and self.dead_mans_switch:
            self.pub.publish(data)

    def manualDriveHandler(self,data):
        if (not self.auto_ctrl) and self.dead_mans_switch:
            self.pub.publish(data)

    def autoCtrlHandler(self,data):
        self.auto_ctrl = data.data
        rospy.loginfo("Switched control to %s", "auto" if self.auto_ctrl else "manual")

    def deadMansSwitchHandler(self,data):
        self.dead_mans_switch = data.data
        self.last_dms_msg = rospy.get_time()
        if not self.dead_mans_switch:
            ack = AckermannDrive()
            ack.steering_angle = 0
            ack.speed = 0
            self.pub.publish(ack)
    
    def spin(self):
        #if no dms message received in a while, then stop truck (if bluetooth disconnects etc)
        while not rospy.is_shutdown():
            if rospy.get_time() - self.last_dms_msg > 0.3 and (self.last_dms_msg != -1):
                ack = AckermannDrive()
                ack.steering_angle = 0
                ack.speed = 0
                self.pub.publish(ack)
            
            rospy.sleep(0.1)

if __name__ == '__main__':
    tm = TruckMaster()
    tm.spin()
