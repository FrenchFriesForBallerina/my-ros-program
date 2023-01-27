#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

device_aadress = 62
registry_address = 17
target_sensor_position = 4.5
vehicle_speed = 0.55
rospy_rate = 20

# constants:
Kp = 0.1
Ki = 0
Kd = 0.1
I = 0

speed = WheelsCmdStamped()
error = 0
last_error = 0

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
        def turn_left():
            speed.vel_left = 0.01
            speed.vel_right = vehicle_speed
        def turn_right():
            speed.vel_right = 0.01
            speed.vel_left = vehicle_speed
        def forward():
            speed.vel_left = vehicle_speed   
            speed.vel_right = vehicle_speed

        def find_proportional_gain(error):
            # kp 0.1 speed 0.55, time 0.52.10
            # kp 0.05 speed 0.55, time 0.53.14
            # kp 0.05 speed 0.6, time 0.53 with errors
            # kp 0.1 speed 0.6, time - lots of errors, too zigzaggy
            # kp 0.075 speed 0.6, time - lots of errors
            # kp 0.075 speed 0.55, time - errors, wheel gets stuck
            # kp 0.08 speed 0.55, time - one error - goes onto neighbor lane
            # kp 0.1 speed 0.53, time - errory
            # kp 0.1 speed 0.5, time - error, not too bad, 0.59

            P = Kp * error
            return P

        def find_integral_gain(error):
            global I
            I = I + (rospy_rate * error * Ki)
            # print("integraL:", I)
            return I

        def find_derivative_gain(error, last_error):
            D = Kd * ((error - last_error)/rospy_rate)
            # print("integraL:", I)
            return D

        def apply_PI(P):
            speed.vel_right = vehicle_speed + P
            speed.vel_left = vehicle_speed - P
        
        global error
        global last_error
        
        last_value = 0
        rate = rospy.Rate(rospy_rate)

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
                else:                   # went right 
                    turn_left()
            if read != 0:
                last_value = read           

            indices = []
            for idx, value in enumerate(bits):
                if value == '1':
                    indices.append(idx + 1)
            
            print(indices)
            
            sum = 0
            average = 0

            for i in indices:
                sum += i
            if len(indices) != 0:
                average = sum / (len(indices)) * 1.0
                print("Error:", error)
                
                last_error = error
                error = target_sensor_position - average

                print("Error:", error)
                print("Last error:", last_error)

                if (left == '0001' and right == '1000'): # is going forward                 print("forward")
                    forward()
                else:
                    P = find_proportional_gain(error)
                    I = find_integral_gain(error)
                    D = find_derivative_gain(error, last_error)

                    print("Integral gain is", I)
                    print("Derivative gain is", D)
                    apply_PI(P+I+D) # if not going forward, then correct

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
