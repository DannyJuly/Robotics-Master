#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from assignment0.msg import TwoInt
from std_msgs.msg import Int16

result = Int16()

def callback(msg):
    result.data = msg.num1 + msg.num2
#	rospy.loginfo(result)

def listener():

    rospy.init_node('adder', anonymous=True)
    pub = rospy.Publisher('sum', Int16, queue_size = 1)
    sub = rospy.Subscriber('numbers', TwoInt, callback)
    r = rospy.Rate(10)
#    rospy.spin()
    while not rospy.is_shutdown():
        pub.publish(result)
        r.sleep()

if __name__ == '__main__':
    listener()
