# Test the coder
## Programming test for software and robotics engineer candidates at General Interfaces GmbH

### Task:
Write a ROS node that calculates the moving maximum (running maximum) over 1000 samples in (soft) realtime of an incoming stream of Uint64s.
Publish your result after processing the incoming number and in addition forward the incoming number to the verifying node.

Add documentation and further adjustments as you see fit.

A rough guide to getting started can be found below.

Have fun and good Success.

####Requirements
* a working ROS distribution
* Indigo or Jade
* Python 2.7

#### Steps:

1. Create either a C++ or Python ROS node
  * You can find guidance at the [ROS wiki](http://wiki.ros.org/ROS/Tutorials)
2. Subscribe to the publisher node
  * topic: /numbers
  * type: Int64
3. Advertise your solution topic
  * topic: /verify
  * type: programming_test/Solution
4. Do your magic...
5. Publish your solution after you have incorporated the latest incoming data point. 
Include (forward) the incoming data point on the *input* slot of the message.
6. Add your node to the launch file.
7. Send your solution to the maintainer


#### License:
Copyright (C) 2015 General Interfaces GmbH

Maintainer: Raphael Dürscheid <mailto:rd@gi.ai>

Program: *Test the coder*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
