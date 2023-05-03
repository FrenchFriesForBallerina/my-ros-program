#!/usr/bin/env python3
import rospy
import time

from sensor_msgs.msg import Range
from smbus2 import SMBus
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped

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


class Drive(DTROS):

    def __init__(self, node_name):
        super(Drive, self).__init__(
            node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.sub = rospy.Subscriber(
            "/weirdbot/front_center_tof_driver_node/range", Range, self.obstacle_callback)
        self.sub = rospy.Subscriber(
            "/weirdbot/left_wheel_encoder_node/tick", WheelEncoderStamped, self.tick_callback)
        self.car = Car(0.2)
        self.pid_controller = PID_Controller(0.1, 0.004, 0.16, 0, 40)

    def obstacle_callback(self, data):
        distance = round(data.range * 100)
        if distance < 25:
            print('I am close: %s cm', distance)
            self.car.obstacle_ahead = True
            # activate the drive around the obstacle part

    def tick_callback(self, data):
        print("TICKSSSSSS", data.data)

    def on_shutdown(self):
        speed = WheelsCmdStamped()
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
                self.car.speed_right_wheel = 0
                self.car.speed_left_wheel = 0
                self.publish_speed()

    def stop_for_03_sec(self):
        self.car.speed_right_wheel = 0
        self.car.speed_left_wheel = 0
        self.publish_speed()
        time.sleep(0.5)

    def move_forward_mid(self):
        self.car.speed_right_wheel = 0.3
        self.car.speed_left_wheel = 0.3
        self.publish_speed()
        time.sleep(0.9)

    def move_forward(self):
        self.car.speed_right_wheel = 0.3
        self.car.speed_left_wheel = 0.3
        self.publish_speed()
        time.sleep(1.2)

    def move_forward_constantly(self):
        self.car.speed_right_wheel = 0.3
        self.car.speed_left_wheel = 0.3
        self.publish_speed()
        # add exit condition here?

    def turn_left(self):
        self.car.speed_right_wheel = 0.9
        self.car.speed_left_wheel = 0
        self.publish_speed()
        time.sleep(0.3)

    def turn_right(self):
        self.car.speed_right_wheel = 0
        self.car.speed_left_wheel = 0.8
        self.publish_speed()
        time.sleep(0.3)

    def publish_speed(self):
        speed = WheelsCmdStamped()
        speed.vel_left = self.car.speed_left_wheel
        speed.vel_right = self.car.speed_right_wheel
        self.pub.publish(speed)

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
            if self.car.turn_at_next_left:
                for _ in range(2):
                    self.car.speed_right_wheel = 0.22
                    self.car.speed_left_wheel = 0.2
                    self.publish_speed()
                    rospy.sleep(0.4)
                self.car.turn_at_next_left = False
            elif self.car.obstacle_ahead:
                print('AVOIDING OBSTACLE')
                self.stop_for_03_sec()
                self.car.obstacle_ahead = False
                """ self.turn_right()
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
                    self.move_forward_constantly() """
            else:
                cruise_control(error, last_error, read,
                               target_sensor_position, self.pid_controller, self.car)

            speed.vel_right = self.car.speed_right_wheel
            speed.vel_left = self.car.speed_left_wheel
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
