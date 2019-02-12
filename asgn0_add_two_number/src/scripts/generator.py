#!/usr/bin/env python

import rospy
import random
from assignment0.msg import TwoInt

def talker():
    pub = rospy.Publisher('numbers', TwoInt, queue_size=1)
    rospy.init_node('generator', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = TwoInt()
    while not rospy.is_shutdown():
#        hello_str = "hello world"
#        rospy.loginfo(hello_str)
#        pub.publish(hello_str)
	msg.num1 = random.randrange(0, 100, 1)
	msg.num2 = random.randrange(0, 100, 1)
#	a = [num1, num2]
#	rospy.loginfo(hello_str)
	pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
