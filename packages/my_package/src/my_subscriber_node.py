#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String
# from std_msgs.msg import WheelsCmdStamped


class MySubscriberNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        """ self.sub = rospy.Subscriber(
            '~/weirdbot/right_wheel_encoder_node/tick', String, self.callback) """

        self.sub = rospy.Subscriber(
            '~/weirdbot/right_wheel_encoder_node/tick', WheelsCmdStamped, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)
        print("hello from subscriber, I hear", data.data)


""" if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin() """

# https://github.com/duckietown/dt-duckiebot-interface/blob/daffy/packages/wheel_encoder/src/wheel_encoder_node.py
