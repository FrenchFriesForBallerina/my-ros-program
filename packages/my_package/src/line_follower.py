#!/usr/bin/env python3
import rospy

from duckietown_msgs.msg import WheelsCmdStamped

speed = WheelsCmdStamped()


def callback(data):
    speed_left_wheel = data.vel_left
    speed_right_wheel = data.vel_right
    rospy.loginfo(speed_left_wheel)
    rospy.loginfo(speed_right_wheel)
    print(type(data.vel_left), data.vel_left)


def stopMoving():
    print('shutting down')
    speed.vel_left = 0
    speed.vel_right = 0
    print(speed.vel_left)
    pub.publish(speed)


def LineFollower():
    while not rospy.is_shutdown():
        rospy.loginfo('hi')
        speed.vel_left = 2
        pub.publish(speed)
        print(speed.vel_left)


if __name__ == '__main__':

    try:
        rospy.on_shutdown(stopMoving)
        rospy.init_node('line_follower', anonymous=True)
        pub = rospy.Publisher(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        sub = rospy.Subscriber(
            '/weirdbot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, callback)

        rate = rospy.Rate(10)

        LineFollower()
        rate.sleep()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
