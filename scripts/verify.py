#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from programming_test.msg import Solution
import numpy as np

class running_maximum:
    """calculate the running maximum for a given window"""
    def __init__(self, windowsize):
        self.array = np.zeros(windowsize)

    def put(self, number_in):
        np.put(self.array, [0], [number_in])
        self.array = np.roll(self.array,1)
        return True

    def get(self):
        return np.amax(self.array)


def callback(data):
    verify(data)

def verify(data):
    verify.r_max.put(data.input)

    maximum = verify.r_max.get()
    if data.solution is maximum:
        rospy.loginfo(rospy.get_caller_id() + "Sum is correct: %d", data.solution)
        return True
    else:
        rospy.loginfo(rospy.get_caller_id() + "Wrong sum, should be: %d", maximum)
        return False
verify.r_max = running_maximum(10)

def listener():

    rospy.init_node('verify')

    rospy.Subscriber("verify", Solution, callback)

    rospy.spin()

if __name__ == '__main__':
    
    listener()
