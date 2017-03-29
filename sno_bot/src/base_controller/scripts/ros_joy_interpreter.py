import rospy
from sensor_msgs.msg import Joy

def callback(joy):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", joy.data)

def listener():

    rospy.init_node('joy_interpreter', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ is '__main__':
    listener()
