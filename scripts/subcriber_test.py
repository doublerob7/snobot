#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


def callback(data):
	twist = Twist()
	pub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=0)
	twist.linear.x = int(data.axes[1] * 5)
	twist.angular.z = int(data.axes[0] * 5)
	pub.publish(twist)
	rospy.loginfo("I heard %s, I published %s", data.axes[0], twist.linear.x)
	
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joy_listener', anonymous=True)

    rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down...")
