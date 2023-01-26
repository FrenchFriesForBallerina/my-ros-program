#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

device_aadress = 62
registry_address = 17

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
            speed.vel_left = 0.0
            speed.vel_right = medium
        def turn_right():
            speed.vel_right = 0.0
            speed.vel_left = medium
        def forward():
            speed.vel_left = medium   
            speed.vel_right = medium
        def proportional(index):
            Kp = (index + 1) * 0.1 * 0.1
            error = 4.5 - index + 1
            P = Kp * error
            speed.vel_right = medium + P
            speed.vel_left = medium - P
        
        last_value = 0
        rate = rospy.Rate(20) # 20Hz - code runs 20 times per second
        # check https://answers.ros.org/question/264812/explanation-of-rospyrate/ 
        slow = 0.25
        medium = 0.50
        fast = 2

        while not rospy.is_shutdown():
            bus = SMBus(1)
            read = bus.read_byte_data(device_aadress,registry_address)
            
            bits_block = bin(read)[2:]
            leading_zeros = 8 - len(bits_block)
            bits = leading_zeros*'0' + bits_block
            left, right = bits[:4], bits[4:]

            print("binary read is", bits)
            print("left is", left)
            print("right is", right)

            if read == 0:
                if last_value < 24:     # went left
                    turn_right()
                else:  # went right 
                    turn_left()
            if read != 0:
                last_value = read          

            if left == '0001' or right == '1000': # is going forward
                forward()
            else:
                if left != '0001' and right != '1000' and '1' in bits:
                    proportional(bits.index('1')) # returns the first found match 
       
            self.pub.publish(speed)
            rate.sleep()
            bus.close()
            print(read)

    def run(self):
        self.simple_track()
            
if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()