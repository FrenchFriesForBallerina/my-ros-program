#!/usr/bin/env python3
import rospy
from rospy import Subscriber
from statistics import mean

from helper_functions import int_to_bitblock
from Car import Car
from Pid_controller import PID_Controller
from my_subscriber_node import MySubscriberNode

from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

import matplotlib.pyplot as plt


sparkfun_device_aadress = 62
sparkfun_registry_address = 17
target_sensor_position = 4.5
vehicle_speed = 0.6
# 0.5  # kp 0.1 great with speed 0.4 but dies on turns, too weak
rospy_rate = 40  # 20
Kp = 0.03105  # 0.025 ei vea kurvides välja with Kd # 0.02 seems to be almost there! with Kd# 0.15 and 0.16 not awful with Kd # starting to get not too awful with 0.07, also 0.035, 0.017 too weak response
# 0.026 tsipa parem, aga kurvid...
# 0.027 sama
# 0.028 peaaegu
# 0.029 nagu hakkab halvemaks minema või tundub?
# 0.03 nagu ikka ei vea välja mõnel kurvil
# 0.032 juba hakkab siksakitama, seega juba palju
# 0.03105 keskmiselt 4 viga raja peale (järske kurve ja hargnemisi arvesse ei võta)

Ki = 0
Kd = 10 * Kp  # .01  # 1
I = 0

""" Kp = rospy.get_param("/p")
ki = rospy.get_param("/i")
kd = rospy.get_param("/d") """

speed = WheelsCmdStamped()
error = 0
last_error = 0

car = Car(vehicle_speed)
pid_controller = PID_Controller(Kp, Ki, Kd, I, rospy_rate)


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        super(MyPublisherNode, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def simple_track(self):
        global error
        global last_error
        rate = rospy.Rate(rospy_rate)

        while not rospy.is_shutdown():
            X = [1, 2, 3, 4, 5]
            Y = [5, 6.4, 6.5, 7.6, 2]
            # index/place where the corresponding value of Y is needed, check the array index must be equal to 3.
            plt.figure()
            plt.plot(X, Y)

            for i in range(len(X)):
                plt.annotate(str(Y[i]), xy=(X[i], Y[i]))

            plt.show()

            print("left speed:", car.speed_left_wheel)
            print("right speed:", car.speed_right_wheel)
            bus = SMBus(1)
            read = bus.read_byte_data(
                sparkfun_device_aadress, sparkfun_registry_address)

            bits_block, indices = int_to_bitblock(read)
            print('\nindices are:', indices)

            if len(indices) != 0:
                last_error = error
                error = target_sensor_position - mean(indices)  # average
                print('error is', error)
                print('bits_block is', bits_block)
                # if bits_block[0] == '1' and bits_block[7] == '0':
                #    car.sharp_left()
                # elif bits_block[0] == '0' and bits_block[7] == '1':
                #    car.sharp_right()
                # else:
                pid_controller.apply_controller(car, error, last_error)

            else:
                print('OFF ROAD')
                car.speed_left_wheel = 0
                car.speed_right_wheel = 0
                """ if last_error < 0:
                    car.sharp_right()
                elif last_error > 0:
                    car.sharp_left() """

            speed.vel_right = car.speed_right_wheel
            speed.vel_left = car.speed_left_wheel
            self.pub.publish(speed)
            rate.sleep()
            bus.close()

    def run(self):
        self.simple_track()


if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    node.run()
    rospy.spin()

# https://se.mathworks.com/help/supportpkg/arduino/ref/arduino-robot-line-follower-application.html
# check https://answers.ros.org/question/264812/explanation-of-rospyrate/
# https://www.youtube.com/watch?v=wkfEZmsQqiA
