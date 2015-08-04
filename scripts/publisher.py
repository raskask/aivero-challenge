#!/usr/bin/env python
# license removed for brevity
import rospy
from random import randint
from std_msgs.msg import Int64

def talker():
    pub = rospy.Publisher('numbers', Int64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(random.randint(0,065105107105))
        pub.publish(counter)
        counter ++;
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass