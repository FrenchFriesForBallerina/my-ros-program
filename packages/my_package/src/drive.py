""" #!/usr/bin/env python3
import rospy
import time

from cruise_control import cruise_control
from Car import Car
from Pid_controller import PID_Controller

from duckietown.dtros import DTROS, NodeType
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

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

    def stopper(self, binary):
        v = 0
        while v < 2:
            time.sleep(0.17)
            v += 1
            if binary == '00000000':
                (car.speed_right_wheel, car.speed_left_wheel) = 0
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

            if binary == '00000000':
                self.stopper(binary)
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
    node = MyPublisherNode(node_name='LF_publisher')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()
 """
