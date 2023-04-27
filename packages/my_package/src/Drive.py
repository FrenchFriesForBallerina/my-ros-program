#!/usr/bin/env python3
import rospy
import time

from sensor_msgs.msg import Range
from smbus2 import SMBus
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped

from cruise_control import cruise_control
from Car import Car
from Pid_controller import PID_Controller

sparkfun_device_address = 62
sparkfun_registry_address = 17

target_sensor_position = 4.5
vehicle_speed = 0.2
rospy_rate = 40

Kp = 0.1
Ki = 0.004
Kd = 0.16
I = 0

speed = WheelsCmdStamped()
error = 0
last_error = 0

car = Car(vehicle_speed)
pid_controller = PID_Controller(Kp, Ki, Kd, I, rospy_rate)


def callback(data):
    distance = round(data.range * 100)
    if distance < 25:
        print('I am close: %s cm', distance)
        car.obstacle_ahead = True
        # activate the drive around the obstacle part


class Drive(DTROS):

    def __init__(self, node_name):
        super(Drive, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.sub = rospy.Subscriber(
            "/weirdbot/front_center_tof_driver_node/range", Range, callback)

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()

    def stopper(self, binary):
        v = 0
        while v < 2:
            time.sleep(0.17)
            v += 1
            if binary == '00000000':
                (car.speed_right_wheel, car.speed_left_wheel) = 0
                self.pub.publish(speed)

    def move_forward_mid(self):
        car.speed_right_wheel = 0.3
        car.speed_left_wheel = 0.3
        speed.vel_right = car.speed_right_wheel
        speed.vel_left = car.speed_left_wheel
        self.pub.publish(speed)
        time.sleep(0.9)

    def move_forward(self):
        car.speed_right_wheel = 0.3
        car.speed_left_wheel = 0.3
        speed.vel_right = car.speed_right_wheel
        speed.vel_left = car.speed_left_wheel
        self.pub.publish(speed)
        time.sleep(1.2)

    def move_forward_constantly(self):
        car.speed_right_wheel = 0.3
        car.speed_left_wheel = 0.3
        speed.vel_right = car.speed_right_wheel
        speed.vel_left = car.speed_left_wheel
        self.pub.publish(speed)

    def turn_left(self):
        car.speed_right_wheel = 0.9
        car.speed_left_wheel = 0
        speed.vel_right = car.speed_right_wheel
        speed.vel_left = car.speed_left_wheel
        self.pub.publish(speed)
        time.sleep(0.3)

    def turn_right(self):
        car.speed_right_wheel = 0
        car.speed_left_wheel = 0.8
        speed.vel_right = car.speed_right_wheel
        speed.vel_left = car.speed_left_wheel
        self.pub.publish(speed)
        time.sleep(0.3)

    def simple_track(self):
        global error
        global last_error

        rate = rospy.Rate(rospy_rate)

        while not rospy.is_shutdown():
            bus = SMBus(1)
            read = bus.read_byte_data(
                sparkfun_device_address, sparkfun_registry_address)

            binary = bin(read)[2:].zfill(8)

            """ if binary == '00000000':
                self.stopper(binary) """
            if car.turn_at_next_left:
                x = 0
                while x < 2:
                    car.speed_right_wheel = 0.22
                    car.speed_left_wheel = 0.2
                    speed.vel_right = car.speed_right_wheel
                    speed.vel_left = car.speed_left_wheel
                    self.pub.publish(speed)
                    rospy.sleep(0.4)
                    x = x + 1
                car.turn_at_next_left = False
            elif car.obstacle_ahead == True:
                print('AVOIDING OBSTACLE')
                self.turn_right()
                self.move_forward()
                self.turn_left()
                self.move_forward_mid()
                self.turn_left()
                read = bus.read_byte_data(
                    sparkfun_device_address, sparkfun_registry_address)
                binary = bin(read)[2:].zfill(8)
                print(binary)
                car.obstacle_ahead = False
                while binary == '00000000':
                    read = bus.read_byte_data(
                        sparkfun_device_address, sparkfun_registry_address)
                    binary = bin(read)[2:].zfill(8)
                    if binary != '00000000':
                        break
                    print(binary)
                    self.move_forward_constantly()
            else:
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
    node = Drive(node_name='Drive_Weirdbot_Drive')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()
