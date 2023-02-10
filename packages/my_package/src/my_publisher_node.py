#!/usr/bin/env python3
import os
import rospy
from csv import writer
import subprocess

from helper_functions import int_to_bitblock
from Car import Car
from pid_controller import *

from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

sparkfun_device_aadress = 62
sparkfun_registry_address = 17
target_sensor_position = 4.5
vehicle_speed = 0.4
rospy_rate = 20

Kp = 0.05
Ki = 0
Kd = 0.02
I = 0

speed = WheelsCmdStamped()
error = 0
last_error = 0

car = Car(vehicle_speed)
pid_controller = PID_Controller(Kp, Ki, Kd, I, rospy_rate)


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def simple_track(self):
        #location = os.environ['REPO_NAME']
        # print(location)
        # c691e634dc2a

        global error
        global last_error
        last_read = ''
        rate = rospy.Rate(rospy_rate)

        while not rospy.is_shutdown():
            bus = SMBus(1)
            read = bus.read_byte_data(
                sparkfun_device_aadress, sparkfun_registry_address)

            bits_block, sum, indices = int_to_bitblock(read)

            if bits_block == '00000000':
                print('OFF ROAD')
                print('bits block is', bits_block)
                print('last read is', last_read)
                if '1' in last_read[0:4]:
                    car.turn_right()
                else:
                    car.turn_left()

            if bits_block != '00000000':
                # this here
                last_read = bits_block
                print('last_read:', last_read)

            if len(indices) != 0:  # else?
                average = sum / (len(indices)) * 1.0

                last_error = error
                error = target_sensor_position - average

                print(bits_block)
                if bits_block == '00011000':
                    print('forward')
                    car.forward()
                else:
                    pid_controller.apply_controller(car, error, last_error)

            speed.vel_right = car.speed_right_wheel
            speed.vel_left = car.speed_left_wheel
            self.pub.publish(speed)
            rate.sleep()
            bus.close()

    def run(self):
        self.simple_track()


if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()

# https://se.mathworks.com/help/supportpkg/arduino/ref/arduino-robot-line-follower-application.html
# check https://answers.ros.org/question/264812/explanation-of-rospyrate/
# https://www.youtube.com/watch?v=wkfEZmsQqiA
