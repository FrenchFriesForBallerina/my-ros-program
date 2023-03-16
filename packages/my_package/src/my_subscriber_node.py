#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String


class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        super(MySubscriberNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        self.sub = rospy.Subscriber(
            '/weirdbot/wheels_driver_node/wheels_cmd', String, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)


if __name__ == '__main__':
    node = MySubscriberNode(node_name='my_subscriber_node')
    rospy.spin()  # subscriber callbacks getting called

# https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    # https://answers.ros.org/question/257361/what-is-the-actual-meaning-of-rosspin/
