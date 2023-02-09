#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

""" 

       #location = os.environ['REPO_NAME']
        #print(location)
        #c691e634dc2a

class MySubscriberNode(DTROS):
    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        #self.sub = rospy.Subscriber('chatter', String, self.callback)
        self.sub = rospy.Subscriber('/weirdbot/left_wheel_encoder_node/tick', String, self.callback)
    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)
        print("tick data:", data.data)

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()

   #print("left wheel ticks", self.left_ticks, type(self.left_ticks))
        #print("right wheel ticks", self.right_ticks)
        #rospy.loginfo('loginfo', self.right_ticks)
        #print("DICT", self.right_ticks.__dict__)

"""

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber('/weirdbot/wheels_driver_node/wheels_cmd', String, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin() # subscriber callbacks getting called

    #https://answers.ros.org/question/257361/what-is-the-actual-meaning-of-rosspin/


# previous
""" #!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct publisher
        self.sub = rospy.Subscriber('chatter', String, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin() # subscriber callbacks getting called

    #https://answers.ros.org/question/257361/what-is-the-actual-meaning-of-rosspin/ """