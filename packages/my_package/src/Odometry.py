""" N_total = 135
N_left = 0
N_right = 0
dright = 0
dleft = 0
R = 3.3
L = 5
X = 0  # Start position
Y = 0  # Start position
OK = 0

R = 21.2  # cm
D = 6.6  # cm 
# n-left - vasaku ratta tickid, N_total - 135, left_ticks - vasaku ratta l'bitud vahemaa
left_ticks = N_left * ((2 * math.pi) / N_total)
# n-parema ratta tickid
right_ticks = N_right * ((2 * math.pi) / N_total)

dleft = R * left_ticks  # vasaku
dright = R * right_ticks
dAlgus = (dright + dleft) / 2
Ok = (dright - dleft)/2*L
xkordinaat = dAlgus * math.cos(Ok)
ykordinaat = dAlgus * math.sin(Ok)
XKordinaat = X + xkordinaat
YKordinaat = Y + ykordinaat
OK = OK + Ok
print("x kordinaat:", XKordinaat)
print("y kordinaat:", YKordinaat)

"""

#!/usr/bin/env python3

import math
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32


class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(
            node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(...)
        self.sub_encoder_ticks_right = rospy.Subscriber(...)
        self.sub_executed_commands = rospy.Subscriber(...)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(...)
        self.pub_integrated_distance_right = rospy.Publisher(...)

        self.log("Initialized")

    def cb_encoder_data(self, wheel, msg):
        """ Update encoder distance information from ticks.
        """

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """


if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
