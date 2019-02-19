#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

# Copyright (C) 2015 General Interfaces GmbH
# Copyright (C) 2019 Aivero AS
# Maintainer: Raphael Dürscheid

# This file is part of *Challenge the coder*.

# *Challenge the coder* is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# *Challenge the coder* is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with *Challenge the coder*.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from numpy import random
from std_msgs.msg import Int64

def talker():
    pub = rospy.Publisher('numbers', Int64, queue_size=10)
    rospy.init_node('publish', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        random_num = random.random_integers(0,065105107105)
        rospy.loginfo(random_num)
        pub.publish(random_num)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
