#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16, String
from smbus2 import SMBus

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

device_aadress = 62
registry_address = 17
target_sensor_position = 4.5

speed = WheelsCmdStamped()


class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

        """     def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown() """

        rate = rospy.Rate(20)  # 20Hz - code runs 20 times per second

        while not rospy.is_shutdown():
            publishNum = 1234
            rospy.loginfo(publishNum)
            self.pub.publish(speed)
            rate.sleep()

    def run(self):
        self.simple_track()


if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.run()
    rospy.spin()

# https://se.mathworks.com/help/supportpkg/arduino/ref/arduino-robot-line-follower-application.html
