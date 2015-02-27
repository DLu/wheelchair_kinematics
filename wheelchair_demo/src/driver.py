#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, PolygonStamped, Point32
from math import sin, cos, pi
from tf.transformations import quaternion_from_euler
import tf

class Drivable:
    def __init__(self, name, pose):
        self.name = name
        self.cmd = None
        self.sub = rospy.Subscriber('/%s/cmd_vel'%name, Twist, self.cb)
        self.pub = rospy.Publisher('/%s_footprint'%name, PolygonStamped, latch=True)
        self.pose = pose

        self.footprint = PolygonStamped()
        self.footprint.header.frame_id = '/%s'%name
        x = .25
        self.footprint.polygon.points.append(Point32( x, x,0))
        self.footprint.polygon.points.append(Point32( x,-x,0))
        self.footprint.polygon.points.append(Point32(-x,-x,0))
        self.footprint.polygon.points.append(Point32(-x, x,0))
        
    def cb(self, msg):
        self.cmd = msg

    def spin_once(self, time):
        if not self.cmd:
            return
        x0,y0,theta0 = self.pose
        vx = self.cmd.linear.x
        vy = self.cmd.linear.y
        vz = self.cmd.angular.z

        x1 = x0 + (vx * cos(theta0) + vy * cos(pi/2 + theta0)) * time
        y1 = y0 + (vx * sin(theta0) + vy * sin(pi/2 + theta0)) * time
        theta1 = theta0 + vz * time

        print self.pose

        self.pose = (x1,y1,theta1)
        print self.pose
        print

    def publish(self, pub):
        pub.sendTransform(self.get_translation(),
                     self.get_quaternion(),
                     rospy.Time.now(), self.name, '/map')    
        self.pub.publish(self.footprint)

    def get_translation(self):
        return (self.pose[0], self.pose[1], 0.0)

    def get_quaternion(self):
        return quaternion_from_euler(0,0,self.pose[2])

rospy.init_node('driver')
wheelchair = Drivable('wheelchair', (0.0, 0.0, 0.0))
offset = rospy.get_offset('offset', 1.0)
robot = Drivable('pr2', (-offset,0.0,0.0))
br = tf.TransformBroadcaster()

R = 10
r = rospy.Rate(R)
while not rospy.is_shutdown():
    robot.spin_once(1.0/float(R))
    robot.publish(br)
    r.sleep()

