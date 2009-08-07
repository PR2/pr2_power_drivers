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

import operator
import select
import time
import os

# returns total as checksum
# input - string
def checksum(st):
    return reduce(operator.add, map(ord, st))
# returns total mod 256 as checksum
# input - string
def checksum256(st):
    return reduce(operator.add, map(ord, st)) % 256
#    return reduce(operator.add, map(ord, st)) % 256

def hex2dec(s):
    """return the integer value of a hexadecimal string s"""
    return int(s, 16)

def readmask(hexstr):
    char = hex2dec(hexstr)
    mask = []
    for i in range(0,8):
        if char & (1 << i):
            mask.append(True)
        else:
            mask.append(False)
    return mask


#todo: automatically set 
# set serial ports to 19200
# the same as doing below
# stty 19200 </dev/ttyUSBX


class powerBattery:
    def __init__(self):
        self.present = False
        self.charging = False
        self.supplying_power = False
        self.w_charge_power = False
        self.power_no_good = False
        self.charge_inhibited = False
        self.voltage = 0.0
        self.current = 0.0
        self.error = False

class powerController:
    def __init__(self):
        self.time_remaining = 0
        self.average_charge = 0
        self.batteries = [powerBattery(),powerBattery(),powerBattery(),powerBattery(),powerBattery(),powerBattery(),powerBattery(),powerBattery()]
        self.latest_system_messages = []

    def new_system_message(self, mess):
        self.latest_system_messages.append(mess)
        while len(self.latest_system_messages) > 10:
            self.latest_system_messages.pop(0)

    def total_current(self):
        total_current = 0.0;
        for batt in self.batteries:
            total_current += batt.current
        return total_current

    def average_voltage(self):
        total_voltage = 0.0;
        count = 0
        for batt in self.batteries:
            if batt.present:
                total_voltage += batt.voltage
                count += 1

        if count == 0:
            return 0.0
        else:
            return total_voltage/float(count)
    
class robotPower:
    def __init__(self):
        print "made robot power"
        self.controllers = [powerController(),powerController(),powerController(),powerController()]

    def print_remaining(self):
        print "___________________________________"
        print "Battery Controller Information:"
        print "-----------------------------------"
        print 'time remaining per battery pack in minutes'
        print '1: %d, 2:%d, 3:%d, 4:%d'%(self.controllers[0].time_remaining,self.controllers[1].time_remaining,self.controllers[2].time_remaining,self.controllers[3].time_remaining)
        print "average charge percentage"
        print '1: %d, 2:%d, 3:%d, 4:%d'%(self.controllers[0].average_charge,self.controllers[1].average_charge,self.controllers[2].average_charge,self.controllers[3].average_charge)
        print "total_current"
        print '1: %f, 2:%f, 3:%f, 4:%f'%(self.controllers[0].total_current(),self.controllers[1].total_current(),self.controllers[2].total_current(),self.controllers[3].total_current())
        print "average_voltage"
        print '1: %f, 2:%f, 3:%f, 4:%f'%(self.controllers[0].average_voltage(),self.controllers[1].average_voltage(),self.controllers[2].average_voltage(),self.controllers[3].average_voltage())
        

        for control in self.controllers:
            if len(control.latest_system_messages) > 0:
                print "latest system messages:"
                for message in control.latest_system_messages:
                    print message



def main():

    os.system('stty 19200 </dev/ttyUSB0')
    os.system('stty 19200 </dev/ttyUSB1')
    os.system('stty 19200 </dev/ttyUSB2')
    os.system('stty 19200 </dev/ttyUSB3')


#f= open('testfile.txt','r')
    f0 = open('/dev/ttyUSB0','r')
    f1 = open('/dev/ttyUSB1','r')
    f2 = open('/dev/ttyUSB2','r')
    f3 = open('/dev/ttyUSB3','r')
# split on %  to string and checksum
# check checksum

    myPow = robotPower()

    start_time = time.time()
    last_time = start_time
    while True:
        current, blah, blah2 = select.select([f0,f1,f2,f3],[],[],1)
        for f in current:
            if f == f0:
                port = 0;
                port_string = 'f0'
            if f == f1:
                port = 1;
                port_string = 'f1'
            if f == f2:
                port = 2;
                port_string = 'f2'
            if f == f3:
                port = 3;
                port_string = 'f3'
            line = f.readline()
            halves = line.split('%')
            if len(halves) != 2:
                #            print 'I did not split on \% correctly'
                continue
            halves[1]=halves[1].strip()
            halves[0]=halves[0].lstrip('$')

#        print line
#    print halves
#    print len(halves)
#    print checksum256(halves[0])
#    print hex2dec(halves[1])
            #    print hex(255)


            # read first char and switch
            message = halves[0]

            # case C controller

            # split on commas
            # first element is controller number
            # for each pair
            # read index  01-07
            # record value
            if len(message) < 2:
                print "error message too short: \"%s\" from \"%s\""%(message, line)
                print "This often indicates a misconfigured serial port check port:"
                print f
                continue
            if message[0] == 'C':
                controller_number = int(message[1])
                #print 'Controller on port %s says:'%(port_string)
                splitmessage = message.split(',')

                #print 'batteries present'
                #print  readmask(splitmessage[2])
                mask = readmask(splitmessage[2])
                for i in range(0,8):
                    myPow.controllers[port].batteries[i].present = mask[i]
                #print 'batteries charging'
                #print  readmask(splitmessage[4])
                mask = readmask(splitmessage[2])
                for i in range(0,8):
                    myPow.controllers[port].batteries[i].charging = mask[i]
                #print 'batteries supplying power to system'
                #print  readmask(splitmessage[6])
                mask = readmask(splitmessage[6])
                for i in range(0,8):
                    myPow.controllers[port].batteries[i].supplying_power = mask[i]
                #print 'batteries which have charge power present'
                #print  readmask(splitmessage[10])
                mask = readmask(splitmessage[10])
                for i in range(0,8):
                    myPow.controllers[port].batteries[i].w_charge_power = mask[i]
                #print 'batteries with "Power no good"'
                #print  readmask(splitmessage[12])
                mask = readmask(splitmessage[12])
                for i in range(0,8):
                    myPow.controllers[port].batteries[i].power_no_good = mask[i]
                #print 'batteries charge inhibited'
                #print  readmask(splitmessage[14])
                mask = readmask(splitmessage[14])
                for i in range(0,8):
                    myPow.controllers[port].batteries[i].charge_inhibited = mask[i]

            else:
                pass

            if message[0] == 'B':
                controller_number = int(message[1])
                battery_number = int(message[2])

                #print 'Battery %d on Controller %d'%(battery_number, controller_number)

                splitmessage = message.split(',')

                for i in range(0,(len(splitmessage)-1)/2):
                    key = splitmessage[i*2+1]
                    value = splitmessage[i*2+2]
#                print 'key %s , value %s'%(splitmessage[i*2+1],splitmessage[i*2+2])
#                print 'key %s , value dec %s'%(splitmessage[i*2+1],hex2dec(splitmessage[i*2+2]))



                    if key == '09':
                        #print 'voltage is %f'%(float(hex2dec(value))/1000.0)
                        myPow.controllers[port].batteries[battery_number].voltage = float(hex2dec(value))/1000.0 


                    if key == '0A':
                        intval = float(hex2dec(value))
                        if intval < 32767:
                            val = intval
                        else:
                            val = (intval - 65636.0)
                        #print 'current is %f'%(val/1000.0)
                        myPow.controllers[port].batteries[battery_number].current = val/1000.0


# case B battery
# split on commas
# first number is battery number
# foreach pair
# lookup index
# record value cast properly so pages 13 to 35

#case S system data
            if message[0] == 'S':
                splitmessage = message.split(',')
                #print 'System Message'
                for i in range(0,(len(splitmessage)-1)/2):
                    key = splitmessage[i*2+1]
                    value = splitmessage[i*2+2]
#            print 'key %s , value %s'%(key,value)
#            print 'key %s , value dec %s'%(key,hex2dec(value))

                    if key == '01':
                        #print 'time until exhaustion = %s minutes'%hex2dec(value)
                        myPow.controllers[port].time_remaining = int(hex2dec(value))
                    if key == '02':
                        #reserved
                        pass

                    if key == '03':
                        #print 'text message to the system %s'%value
                        myPow.controllers[port].new_system_message(value)

                    if key == '04':
                        #print 'average charge percent %d'% hex2dec(value)
                        myPow.controllers[port].average_charge = int(hex2dec(value))

        #print time.time()
        increment = 1.0
        if time.time() - last_time > increment:
            last_time = last_time + increment
            myPow.print_remaining()


main()
