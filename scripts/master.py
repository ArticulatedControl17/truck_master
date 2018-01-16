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

    def deadMansSwitchHandler(self,data):
        self.dead_mans_switch = data.data
        self.last_dms_msg = rospy.get_time()
    
    def spin(self):
        #if no dms message received in a while, then stop truck (if bluetooth disconnects etc)
        while not rospy.is_shutdown():
            if rospy.get_time() - self.last_dms_msg > 0.3 and (self.last_dms_msg != -1):
                print "master: no message in 0.3 sec, stopping truck"
                self.dead_mans_switch = False
            
            rospy.sleep(0.1)

if __name__ == '__main__':
    tm = TruckMaster()
    tm.spin()
