/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "power_state_estimator.h"

#include <stdlib.h>

#include "ros/ros.h"

using namespace std;

// PowerObservable

ros::PowerObservable::PowerObservable() { }

ros::PowerObservable::PowerObservable(const vector<ros::BatteryObservable>& batteries) : batteries_(batteries) { }

const vector<ros::BatteryObservable>& ros::PowerObservable::getBatteries() const { return batteries_; }

int ros::PowerObservable::getAcCount() const
{
    int ac_count = 0;
    for (unsigned int i = 0; i < batteries_.size(); i++)
        if (batteries_[i].isAcPresent())
            ac_count++;

    return ac_count;
}

float ros::PowerObservable::getTotalPower() const
{
    float total_power = 0.0f;
    for (unsigned int i = 0; i < batteries_.size(); i++)
        total_power += batteries_[i].getPower();

    return total_power;
}

float ros::PowerObservable::getMinVoltage() const
{
    float min_voltage = 9999.9f;
    for (unsigned int i = 0; i < batteries_.size(); i++)
        min_voltage = min(min_voltage, batteries_[i].getVoltage());

    return min_voltage;
}

// BatteryObservable

ros::BatteryObservable::BatteryObservable(bool ac_present, float voltage, float current, unsigned int remaining_capacity, unsigned int time_to_empty, unsigned int time_to_full)
    : ac_present_(ac_present), voltage_(voltage), current_(current), remaining_capacity_(remaining_capacity), time_to_empty_(time_to_empty), time_to_full_(time_to_full)
{
}

bool         ros::BatteryObservable::isAcPresent()          const { return ac_present_;         }
float        ros::BatteryObservable::getVoltage()           const { return voltage_;            }
float        ros::BatteryObservable::getCurrent()           const { return current_;            }
unsigned int ros::BatteryObservable::getRemainingCapacity() const { return remaining_capacity_; }
unsigned int ros::BatteryObservable::getTimeToEmpty()       const { return time_to_empty_;      }
unsigned int ros::BatteryObservable::getTimeToFull()        const { return time_to_full_;       }

float ros::BatteryObservable::getPower() const
{
    return voltage_ * current_;
}

// FuelGaugePowerEstimator

std::string ros::FuelGaugePowerStateEstimator::getMethodName() const
{
    return "fuel gauge";
}

ros::PowerStateEstimate ros::FuelGaugePowerStateEstimator::estimate(const ros::PowerObservable& power)
{
    // Get the number of batteries charging, the minimum capacity of the batteries, and the maximum & minimum time remaining
    int          ac_count     = 0;
    unsigned int min_capacity = 0;
    unsigned int max_ttf      = 0;
    unsigned int min_tte      = 0;
    for (unsigned int i = 0; i < power.getBatteries().size(); i++)
    {
        const BatteryObservable& b = power.getBatteries()[i];

        bool         ac_present = b.isAcPresent();
        unsigned int rsc        = b.getRemainingCapacity();
        unsigned int tte        = b.getTimeToEmpty();
        unsigned int ttf        = b.getTimeToFull();

        if (ac_present)
            ac_count++;

        if ((tte != 65535) && (i == 0 || tte < min_tte))
            min_tte = tte;
        if ((ttf != 65535) && (i == 0 || ttf > max_ttf))
            max_ttf = ttf;
        if (i == 0 || rsc < min_capacity)
            min_capacity = rsc;
    }

    if (min_capacity == 999)
        min_capacity = 0;

    // If charging, use the maximum time-to-full, otherwise use the minimum time-to-empty.
    unsigned int  time_remaining_mins = (ac_count > 0 ? max_ttf : min_tte);
    ros::Duration time_remaining      = ros::Duration().fromSec(time_remaining_mins * 60);

    ros::PowerStateEstimate ps;
    ps.time_remaining    = time_remaining;
    ps.relative_capacity = min_capacity;

    return ps;
}

// AdvancedPowerEstimator

std::string ros::AdvancedPowerStateEstimator::getMethodName() const
{
    return "advanced";
}

ros::PowerStateEstimate ros::AdvancedPowerStateEstimator::estimate(const ros::PowerObservable& power)
{
    ros::Duration time_remaining     = ros::Duration().fromSec(0 * 60);
    unsigned int  remaining_capacity = 0;

    ROS_ERROR("not implemented");

    ros::PowerStateEstimate ps;
    ps.time_remaining    = time_remaining;
    ps.relative_capacity = remaining_capacity;

    return ps;
}

