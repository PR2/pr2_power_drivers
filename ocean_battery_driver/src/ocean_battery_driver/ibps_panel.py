
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

PKG = 'ocean_battery_driver'

import roslib
roslib.load_manifest(PKG)

import sys
import rospy
from pr2_msgs.msg import PowerState


import wx
from wx import xrc

import threading, time

class BatteryPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent, wx.ID_ANY)
        
        self._mutex = threading.Lock()
        
        xrc_path = roslib.packages.get_pkg_dir(PKG) + '/ui/battery_status_panel.xrc'
        
        self._xrc = xrc.XmlResource(xrc_path)
        self._real_panel = self._xrc.LoadPanel(self, 'BatteryStatusPanel')
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self._real_panel, 1, wx.EXPAND)
        self.SetSizer(sizer)

        rospy.Subscriber("power_state", PowerState, self.message_callback)
        
        self.power_text = xrc.XRCCTRL(self._real_panel, 'm_powerField')
        self.time_field = xrc.XRCCTRL(self._real_panel, 'm_timeField')
        self.status_text = xrc.XRCCTRL(self._real_panel, 'm_statusField')
        self.capacity_gauge = xrc.XRCCTRL(self._real_panel, 'm_capacityGauge')
        self.capacity_text = xrc.XRCCTRL(self._real_panel, 'm_capacityText')

        self.power_text.SetEditable(False)
        self.time_field.SetEditable(False)
        self.status_text.SetEditable(False)
        self.capacity_text.SetEditable(False)

        # Start a timer to check for timeout
        self.timeout_interval = 10 
        self.last_message_time = rospy.get_time()
        self.timer = wx.Timer(self, 1)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self.start_timer()

        self._messages = []
        
    def message_callback(self, message):
        self._mutex.acquire()
        
        self._messages.append(message)
        self.last_message_time = rospy.get_time()
        
        self._mutex.release()
        
        wx.CallAfter(self.new_message)
        
    def new_message(self):
        self._mutex.acquire()
        for message in self._messages:
            self.start_timer()
            self.power_text.Enable()
            self.time_field.Enable()
            self.status_text.Enable()
            self.capacity_text.Enable()
            self.status_text.SetValue('%s'%message.prediction_method)
            self.power_text.SetValue('%.2f Watts'%message.power_consumption)
            self.capacity_gauge.SetValue(message.relative_capacity)
            self.capacity_text.SetValue('%d%%'%message.relative_capacity)
            time_remaining = message.time_remaining.sec / 60
            if message.AC_present > 0:
              self.time_field.SetValue('%u minutes to full'%(time_remaining))
              self.time_field.SetBackgroundColour("White")
            else:
              self.time_field.SetValue('%u minutes to empty'%(time_remaining))
              if time_remaining > 10:
                  self.time_field.SetBackgroundColour("Light Green")
              elif time_remaining > 5:
                  self.time_field.SetBackgroundColour("Orange")
              else:
                  self.time_field.SetBackgroundColour("Red")
                                
        self._messages = []
        
        self._mutex.release()
        
        self.Refresh()
        
    def start_timer(self):
      # Set a timer to expire one second after we expect to timeout. This
      # way a timer event happens at most once every second if the
      # simulation is in slow motion, and we are at most one second late if
      # the simulation is happening in real time.
      interval = rospy.get_time() - self.last_message_time;
      sleep_time = 1000 * (self.timeout_interval - interval + 1);
      self.timer.Start(sleep_time, True)
      #print 'start_timer %f\n'%sleep_time

    def on_timer(self, event):
      self._mutex.acquire()

      interval = rospy.get_time() - self.last_message_time
      
      #print 'on_timer %f %f %f\n'%(rospy.get_time(), self.last_message_time, interval)

      # Consider that we have timed out after 5 seconds of ros time, or if
      # the ros time jumps back (in that case something fishy just happened
      # and the previously displayed value is most likely stale).
      if interval > self.timeout_interval or interval < 0:
        #print 'timeout\n'
        self.power_text.Disable()
        self.time_field.Disable()
        self.status_text.Disable()
        self.time_field.SetBackgroundColour("White")
        self.capacity_text.Disable()
      else:
        self.start_timer()

      self._mutex.release()
      self.Refresh()
