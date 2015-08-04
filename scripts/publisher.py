#!/usr/bin/env python
# license removed for brevity
import rospy
from numpy import random
from std_msgs.msg import Int64

def talker():
    pub = rospy.Publisher('numbers', Int64, queue_size=10)
    rospy.init_node('publish', anonymous=True)
    rate = rospy.Rate(0.4) # 10hz
    while not rospy.is_shutdown():
        random_num = random.randint(0,065105107105)
        rospy.loginfo(random_num)
        pub.publish(random_num)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass