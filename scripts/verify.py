#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from programming_test import Solution

import numpy

class running_maximum:
    """calculate the running maximum for a given window"""
    def __init__(self, windowsize):
        self.array = np.zeros(windowsize)

    def put(number_in):
        np.insert(self.array, 0, number_in)
        np.roll(self.array,1)
        
    def get:
        return np.amax(self.array)

def callback(data):
    answer = verify(data)
    if answer is True
        rospy.loginfo(rospy.get_caller_id() + "Sum is correct: %s", data.solution)
    else
        rospy.loginfo(rospy.get_caller_id() + "Wrong sum, should be: %s", answer)


def verify(data):
    r_max.put(data.input)
    maximum = r_max.get
    if data.solution is maximum:
        return True
    else
        return maximum

r_max = running_maximum(10)

def listener:

    rospy.init_node('verify', anonymous=True)

    rospy.Subscriber("verify_solution", Solution, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    listener()
