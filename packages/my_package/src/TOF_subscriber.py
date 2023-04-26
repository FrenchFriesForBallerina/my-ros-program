#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range


def callback(data):
    distance = round(data.range * 100)
    if distance < 15:
        print('I am close: %s cm', distance)


def listener():
    rospy.init_node('TOF_subscriber', anonymous=True)
    rospy.Subscriber(
        "/weirdbot/front_center_tof_driver_node/range", Range, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
