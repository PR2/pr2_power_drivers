#! /usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************

import roslib
roslib.load_manifest('ocean_battery_driver')
import rospy
from robot_msgs.msg import BatteryState
import os, sys

class BatteryNotifier:
  def __init__(self, notify_limit, state_topic, email_addresses, robot_name, mail_program):
    self.notify_limit = notify_limit
    rospy.init_node("battery_notifier", anonymous=True)
    rospy.Subscriber(state_topic, BatteryState, self.update)
    self.email_addresses = email_addresses
    self.mail_program = mail_program
    self.robot_name = robot_name
    self.mail_sent = False

  def update(self, state):
    if(state.energy_capacity == 0 or (state.energy_remaining / state.energy_capacity) <= self.notify_limit):
      if not self.mail_sent and state.power_consumption < 0.0: 
        self.sendEmail()
        self.mail_sent = True
    else:
      self.mail_sent = False

  def sendEmail(self):
    mail_string = "To: "
    for address in self.email_addresses: mail_string += address + ", "
    mail_string += "\nFrom: %s@willowgarage.com" % self.robot_name
    mail_string += "\nSubject: Battery level on robot below %.2f" % self.notify_limit
    mail_string += "\n\nMy battery level has fallen below the notification level. You should probably think about plugging me in. \n\nThanks much,\n%s" % self.robot_name

    #since the robot's have mail servers installed on them... we'll just pipe out our e-mail
    pipe = os.popen("%s -t" % self.mail_program, 'w')
    pipe.write(mail_string)
    pipe.close()

  def run(self):
    rospy.spin()

if __name__ == '__main__':
  if len(sys.argv) < 2:
    notifier = BatteryNotifier(0.2, "battery_state", ["eitan@willowgarage.com"], "pre", "/usr/sbin/sendmail")
  else:
    notifier = BatteryNotifier(0.2, "battery_state", sys.argv, "pre", "/usr/sbin/sendmail")

  notifier.run()
