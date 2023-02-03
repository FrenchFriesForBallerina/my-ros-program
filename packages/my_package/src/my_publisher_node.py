#!/usr/bin/env python3
import os, rospy
from csv import writer
import subprocess

from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped

sparkfun_device_aadress = 62
sparkfun_registry_address = 17
target_sensor_position = 4.5
vehicle_speed = 0.4
rospy_rate = 20

# constants:
Kp = 0.05 # 0.065 # used to be o.1
Ki = 0 # 0.0001 pane piirangud peale
Kd = 0.02 #0.04
I = 0

speed = WheelsCmdStamped()
error = 0
last_error = 0

def update_csv(lap_time, vehicle_speed, rate, P, I, D):
    print("writing into file")
    List = [lap_time, vehicle_speed, rate, P, I, D]
    with open('results.csv', 'a') as f_object:
        writer_object = writer(f_object)
        writer_object.writerow(List)
        f_object.close()

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
        #location = os.environ['REPO_NAME']
        #print(location)
        #c691e634dc2a

        global error
        global last_error
        last_value = 0
        rate = rospy.Rate(rospy_rate)

        def turn_left():
            speed.vel_left = 0.001 * vehicle_speed
            speed.vel_right = vehicle_speed
        def turn_right():
            speed.vel_right = 0.001 * vehicle_speed
            speed.vel_left = vehicle_speed 
        def forward():
            speed.vel_left = vehicle_speed  
            speed.vel_right = vehicle_speed

        def apply_controller(error, last_error):
            global I
            P = Kp * error
            I = I + (rospy_rate * error * Ki)
            D = Kd * ((error - last_error)/rospy_rate)
            PID = P + I + D
            speed.vel_right = vehicle_speed + PID
            speed.vel_left = vehicle_speed - PID
        
        while not rospy.is_shutdown():
            bus = SMBus(1)
            read = bus.read_byte_data(sparkfun_device_aadress,sparkfun_registry_address)
            print('read is', read)       
            bits_block = bin(read)[2:] #'01100000'
            leading_zeros = 8 - len(bits_block)
            bits = leading_zeros*'0' + bits_block
            
            #left, right = bits[:4], bits[4:]      

            indices = []
            for idx, value in enumerate(bits): 
                if value == '1':
                    print('bit index is', idx)
                    indices.append(idx + 1)
            
            print(indices)
            
            sum = 0
            average = 0

            for i in indices:
                sum += i

            if bits_block == '00000000':
                print('OFF ROAD')
                if 1 in last_value[4:]:    
                    turn_right()
                    print('turning right')
                else:                   
                    turn_left()
                    print('turning left')

            if bits_block != '00000000':
                last_value = read   

            if len(indices) != 0:
                average = sum / (len(indices)) * 1.0
                
                last_error = error
                error = target_sensor_position - average

                if bits_block == '00011000': # is going forward                 print("forward")
                    forward()
                else:
                    apply_controller(error, last_error) # if not going forward, then correct

            print("update csv values", 0, vehicle_speed, rospy_rate, Kp, Ki, Kd)    
            update_csv(0, 0, 0, Kp, Ki, Kd)


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
