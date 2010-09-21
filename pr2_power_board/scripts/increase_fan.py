#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


## Simple program to send commands to the power board manually

PKG = 'pr2_power_board' # this package name

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string

import rospy
from pr2_power_board.srv import *

def set_fan_speed(speed):
    try:
        # create a handle to the add_two_ints service
        control = rospy.ServiceProxy('power_board/control', PowerBoardCommand)
        
        # simplified style
        resp1 = control(0, 0, 'fan', speed)
        return resp1.retval
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return 255 

if __name__ == "__main__":
    rospy.wait_for_service('power_board/control', 5)
    
    rospy.init_node('power_fan_increase')

    my_rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        retcode = set_fan_speed(100)

        if not retcode == 0:
            rospy.logwarn('Unable to set fan speed. Service returned %d' % retcode)

        my_rate.sleep()
