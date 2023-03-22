#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped


def callback():
    rospy.loginfo('I heard you')


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber(
        '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, callback)


if __name__ == '__main__':
    listener()
    rospy.spin()  # subscriber callbacks getting called

# https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# https://answers.ros.org/question/257361/what-is-the-actual-meaning-of-rosspin/
