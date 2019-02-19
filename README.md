# Test the coder
## Programming test for software and robotics engineer candidates at Aivero AS

### Task:
Write a ROS node that calculates the moving maximum (running maximum) over 1000 samples in (soft) realtime of an incoming stream of Uint64s.
Publish your result after processing the incoming number and in addition forward the incoming number to the verifying node.

Add documentation and further adjustments as you see fit.

A rough guide to getting started can be found below.

Have fun and good Success.

#### Requirements
* a working ROS distribution installed on your system (https://wiki.ros.org/ROS/Installation)
  * Ubuntu 16.04 or 18.04 (USE THEM or a ROS docker container)
  * Kinetic (on 16.04) or Melodic (on 18.04)
* Python 2.7

#### Steps:

1. Create either a C++ or Python ROS node
  * You can find guidance at the [ROS wiki](http://wiki.ros.org/ROS/Tutorials)
2. Write a callback that subscribes to the publisher node
  * topic: /numbers
  * type: Int64
3. Advertise your solution topic
  * topic: /verify
  * type: programming_test/Solution
4. In the callback from (2) do your magic... and ...
5. Publish your solution after you have incorporated the latest incoming data point (Publish on every single run of the callback).
Include (forward) the incoming data point on the *input* slot of the message (see **Solution.msg** for the message definition).
6. Add your node to the launch file.
7. Send your solution to the maintainer


#### License:
Copyright (C) 2015 General Interfaces GmbH
Copyright (C) 2019 Aivero AS

Maintainer: Raphael Dürscheid

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
