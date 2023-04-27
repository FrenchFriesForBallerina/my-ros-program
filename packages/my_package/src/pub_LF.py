#!/usr/bin/env python3
import rospy

from duckietown_msgs.msg import WheelsCmdStamped
from smbus2 import SMBus
from std_msgs.msg import String

#from helper_functions import int_to_bitblock
speed = WheelsCmdStamped()

sparkfun_device_aadress = 62
sparkfun_registry_address = 17

target_sensor_position = 4.5
vehicle_speed = 0.5
rospy_rate = 40


def callback(data):
    speed_left_wheel = data.vel_left
    speed_right_wheel = data.vel_right
    rospy.loginfo(speed_left_wheel)
    rospy.loginfo(speed_right_wheel)
    print(type(data.vel_left), data.vel_left)


def stopMoving():
    speed.vel_left = 0
    speed.vel_right = 0
    print('stopping')
    # pub_wheels.publish(speed)


def LineFollower():
    while not rospy.is_shutdown():
        bus = SMBus(1)
        read = bus.read_byte_data(
            sparkfun_device_aadress, sparkfun_registry_address)

        print('read:', read)
        #bits_block = int_to_bitblock(read)
        #print('bits_block:', bits_block)
        #speed.vel_left = speed
        #speed.vel_right = speed
        # pub_wheels.publish(speed)
        bus.close()


if __name__ == '__main__':

    try:
        # rospy.on_shutdown(stopMoving)
        rospy.init_node('pub_LF')

        """ pub_wheels = rospy.Publisher(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10) """

        pub_sparkfun_read = rospy.Publisher(
            '/weirdbot/pub_LF', String, queue_size=10)

        pub_sparkfun_read.publish("hi there")

        """ sub_wheels = rospy.Subscriber(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, callback)
        """
        rate = rospy.Rate(10)

        LineFollower()
        rate.sleep()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
