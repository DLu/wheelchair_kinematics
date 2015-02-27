#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist

class WheelchairDriver:
    def __init__(self):
        self.offset = rospy.get_param('/offset', 1.0)
        self.pub = rospy.Publisher('/pr2/cmd_vel', Twist)
        self.sub = rospy.Subscriber('/wheelchair/cmd_vel', Twist, self.cb)

    def cb(self, msg):
        # Edit the twist message here
        self.pub.publish(msg)

rospy.init_node('wheelchair_driver')
x = WheelchairDriver()
rospy.spin()
