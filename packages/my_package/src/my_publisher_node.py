#!/usr/bin/env python3
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

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
        last_value = 0
        rate = rospy.Rate(20) # 20Hz
        slow = 0.25
        medium = 0.50
        fast = 2
        while not rospy.is_shutdown():
            bus = SMBus(1)
            read = bus.read_byte_data(62,17)
            if read == 8 or read == 16 or read == 24: # FORWARD
                speed.vel_left = fast   
                speed.vel_right = fast
            elif 0 < read < 8 or read == 12:  # STEER LEFT 
                speed.vel_left = fast
                speed.vel_right = speed.vel_right * 0.8
            elif 24 < read < 255:      # STEER RIGHT
                speed.vel_right = fast
                speed.vel_left = speed.vel_right * 0.8
            else:
                speed.vel_left = 0
                speed.vel_right = 0

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


 # previous:
""" #!/usr/bin/env python3
from smbus2 import SMBus
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class MyPublisherNode(DTROS):

    # Open i2c bus 1 and read one byte from address 80, offset 0

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.pub = rospy.Publisher('chatter', String, queue_size=10)

    def run(self):
        bus = SMBus(1)  #1
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            b = bus.read_byte_data(17, 62) #2
            print(b)    #3
            #message = "Hello from %s" % os.environ['VEHICLE_NAME']
            #rospy.loginfo("Publishing message: '%s'" % message)
            rospy.loginfo("byte data is: '%s'" % b)
            #self.pub.publish(message)
            self.pub.publish(b)
            rate.sleep()
        bus.close() # 4

    

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node

   

    node.run()
    # keep spinning
    rospy.spin()
     """

