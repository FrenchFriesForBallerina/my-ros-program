#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String


class RightTicksNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(RightTicksNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber(
            # '/weirdbot/wheels_driver_node/wheels_cmd', String, self.callback)
            '/weirdbot/right_wheel_encoder_node/tick', String, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)
        print("im in right wheel subscriber")


""" if __name__ == '__main__':
    right_ticks_node = RightTicksNode(
        node_name='right_ticks_node')
    rospy.spin()
 """
