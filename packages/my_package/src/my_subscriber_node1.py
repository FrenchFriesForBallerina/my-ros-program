#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import String


class LeftTicksNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LeftTicksNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        """ self.sub = rospy.Subscriber(
            # '/weirdbot/wheels_driver_node/wheels_cmd', String, self.callback)
            '/weirdbot/left_wheel_encoder_node/tick', String, self.callback)
 """
        self.sub = rospy.Subscriber(
            # '/weirdbot/wheels_driver_node/wheels_cmd', String, self.callback)
            '/weirdbot/left_wheel_encoder_node/tick', String, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)
        print("im in left wheel subscriber")


""" if __name__ == '__main__':
    left_ticks_node = LeftTicksNode(node_name='left_ticks_node')
    rospy.spin()  # subscriber callbacks getting called """
