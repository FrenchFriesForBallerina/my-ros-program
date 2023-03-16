#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16


def talker():
    pub = rospy.Publisher('chatter', Int16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publishNum = 1234
        rospy.loginfo('data is being sent', publishNum)
        pub.publish(publishNum)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


# https: // wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber % 28python % 29
