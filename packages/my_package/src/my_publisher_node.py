#!/usr/bin/env python3
import rospy

from cruise_control import cruise_control
from Car import Car
from Pid_controller import PID_Controller

from duckietown.dtros import DTROS, NodeType
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

sparkfun_device_aadress = 62
sparkfun_registry_address = 17
target_sensor_position = 4.5
vehicle_speed = 0.5  # 0.6
rospy_rate = 40

# speed 0.6 rospy rate 40, kp 0.525 ja teised 0 sõidab (ristmikel muidugi pea)
Kp = 0.4
# speed 0.6 rospy rate 40, kp 1.2 ja kp 1.5 ja teised 0 - tõmbleb koha peal
# sama, aga 0.95 - tõmbleb ja liigub edasi
Ki = 0
Kd = 0  # 10 * Kp
I = 0

speed = WheelsCmdStamped()
error = 0
last_error = 0

turn_left_at_junction = False
turn_right_at_junction = False
roadsign_first_detection = False
roadsign_confirmed = False

car = Car(vehicle_speed)
pid_controller = PID_Controller(Kp, Ki, Kd, I, rospy_rate)

# node - sub v pub v ( sub ja pub)
# topic - nt  '/weirdbot/wheels_driver_node/wheels_cmd'
# message - pub/sub saadetud


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
            bus = SMBus(1)
            read = bus.read_byte_data(
                sparkfun_device_aadress, sparkfun_registry_address)

            cruise_control(error, last_error, read,
                           target_sensor_position, pid_controller, car)

            speed.vel_right = car.speed_right_wheel
            speed.vel_left = car.speed_left_wheel
            self.pub.publish(speed)
            rate.sleep()
            bus.close()

    def run(self):
        self.simple_track()


if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()

# https://se.mathworks.com/help/supportpkg/arduino/ref/arduino-robot-line-follower-application.html
# check https://answers.ros.org/question/264812/explanation-of-rospyrate/
# https://www.youtube.com/watch?v=wkfEZmsQqiA
