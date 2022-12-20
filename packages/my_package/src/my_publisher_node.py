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
        def correct_to_right():
            speed.vel_right = 0.0
            speed.vel_left = medium
        def correct_to_left():
            speed.vel_left = 0.0
            speed.vel_right = medium
        
        last_value = 0
        rate = rospy.Rate(20) # 20Hz
        slow = 0.25
        medium = 0.50
        fast = 2
        while not rospy.is_shutdown():
            bus = SMBus(1)
            read = bus.read_byte_data(device_aadress,registry_address)

            if read != 0:
                last_value = read
            
            if read == 0:
                if last_value < 24:     # went left
                    correct_to_right()
                elif last_value >= 24:  # went right 
                    correct_to_left()

            if read == 8 or read == 16 or read == 24: # FORWARD
                speed.vel_left = medium   
                speed.vel_right = medium
            elif 0 < read < 8 or read == 12:  # STEER LEFT 
                speed.vel_left = medium
                speed.vel_right = speed.vel_right * 0.1
            elif 24 < read < 255:      # STEER RIGHT
                speed.vel_right = medium
                speed.vel_left = speed.vel_right * 0.1
       
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
