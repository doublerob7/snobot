#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

"""
INPUT MAPPING:

Table of index number of /joy.buttons:

Index	Button name on the actual controller
0		A
1		B
2		X
3		Y
4		LB
5		RB
6		back
7		start
8		power
9		Button stick left
10		Button stick right

Table of index number of /joy.axis:

Index	Axis name on the actual controller
0		Left/Right Axis stick left
1		Up/Down Axis stick left
2		LT
3		Left/Right Axis stick right
4		Up/Down Axis stick right
5		RT
6		cross key left/right
7		cross key up/down 

"""


def callback(data):
	twist = Twist()
	pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
	twist.linear.x = int(data.axes[1] * 64)
	twist.angular.z = int(-data.axes[0] * 64)
	pub.publish(twist)
	#rospy.loginfo("I heard %s, I published %s", data.axes[0], twist.linear.x)
	
    
def node():
    rospy.init_node('joy_listener', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down...")

