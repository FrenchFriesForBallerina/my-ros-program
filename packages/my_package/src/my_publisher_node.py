#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

device_aadress = 62
registry_address = 17
target_sensor_position = 4.5
sensor_reading_weights = [16, 8, 4, 2, -2, -4, -8, -16] 

speed = WheelsCmdStamped()

class MyPublisherNode(DTROS):
    
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        
    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        rospy.on_shutdown()
        
    def simple_track(self):
        def steer_left():
            speed.vel_left = medium
            speed.vel_right = speed.vel_right * 0.1
        def steer_right():
            speed.vel_right = medium
            speed.vel_left = speed.vel_right * 0.1
        def turn_left():
            speed.vel_left = 0.01
            speed.vel_right = medium
        def turn_right():
            speed.vel_right = 0.01
            speed.vel_left = medium
        def forward():
            speed.vel_left = medium   
            speed.vel_right = medium

        def proportional(index):
            Kp = index * 0.02

            # with 0.02 and speed 0.55: time 0.53 with errors
            # with 0.022 and speed 0.55: time 0.54 with errors
            # with 0.022 and speed 0.525: errors!
            # with 0.02 and speed 0.525: time
            
            error = target_sensor_position - index
            P = Kp * error
            speed.vel_right = medium + P
            speed.vel_left = medium - P
        
        last_value = 0
        rate = rospy.Rate(20) # 20Hz - code runs 20 times per second
        # check https://answers.ros.org/question/264812/explanation-of-rospyrate/ 
        slow = 0.16
        medium = 0.55
        fast = 2

        while not rospy.is_shutdown():
            bus = SMBus(1)
            read = bus.read_byte_data(device_aadress,registry_address)
            
            bits_block = bin(read)[2:]
            leading_zeros = 8 - len(bits_block)
            bits = leading_zeros*'0' + bits_block
            
            
            left, right = bits[:4], bits[4:]

            if read == 0:
                if last_value < 24:     # went left
                    turn_right()
                else:  # went right 
                    turn_left()
            if read != 0:
                last_value = read           

            # calculate deviation from the centre
            # find ones in bits
            indices = []
            for idx, value in enumerate(bits):
                if value == '1':
                    indices.append(idx + 1)
            
            print(indices)
            
            sum = 0
            shift = 0
            for i in indices:
                sum += i
            if len(indices) != 0:
                shift = sum / (len(indices)) * 1.0

                if (left == '0001' and right == '1000'): # is going forward                 print("forward")
                    forward()
                else:
                    proportional(shift)

            self.pub.publish(speed)
            rate.sleep()
            bus.close()
            #print(read)

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