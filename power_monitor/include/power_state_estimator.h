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

#ifndef POWER_MONITOR_POWER_STATE_ESTIMATOR_H
#define POWER_MONITOR_POWER_STATE_ESTIMATOR_H

#include <stdlib.h>

#include "ros/ros.h"

namespace ros
{

class BatteryObservable;

/**
 * Stores information about the power system (all batteries) used to estimate the power state.
 */
class PowerObservable
{
public:
    PowerObservable();
    PowerObservable(const std::vector<BatteryObservable>& batteries);

    const std::vector<BatteryObservable>& getBatteries() const;

    int   getAcCount() const;
    float getTotalPower() const;
    float getMinVoltage() const;

private:
    std::vector<BatteryObservable> batteries_;
};

/**
 * Stores information about a single battery used to estimate the power state.
 */
class BatteryObservable
{
public:
    BatteryObservable(bool ac_present, float voltage, float current, unsigned int remaing_capacity, unsigned int time_to_empty, unsigned int time_to_full);

    bool         isAcPresent() const;
    float        getVoltage() const;
    float        getCurrent() const;
    unsigned int getRemainingCapacity() const;
    unsigned int getTimeToEmpty() const;
    unsigned int getTimeToFull() const;

    float getPower() const;

private:
    bool         ac_present_;
    float        voltage_;
    float        current_;
    unsigned int remaining_capacity_;
    unsigned int time_to_empty_;
    unsigned int time_to_full_;
};

struct PowerStateEstimate
{
    ros::Duration time_remaining;
    unsigned int  relative_capacity;
};

/**
 * PowerStateEstimator takes a PowerObservable and produces a PowerStateEstimate.
 */
class PowerStateEstimator
{
public:
    virtual std::string        getMethodName() const = 0;
    virtual PowerStateEstimate estimate(const PowerObservable& power) = 0;
};

/** A simple power state estimator which uses the minimum & maximum time
  * remaining and the minimum remaining capacity reported by the battery servers.
  */
class FuelGaugePowerStateEstimator : public PowerStateEstimator
{
public:
    std::string        getMethodName() const;
    PowerStateEstimate estimate(const PowerObservable& power);
};

/** A more advanced power state estimator which takes the history of the
  * battery state into account.
  * @todo: implement
  */
class AdvancedPowerStateEstimator : public PowerStateEstimator
{
public:
    std::string        getMethodName() const;
    PowerStateEstimate estimate(const PowerObservable& power);
};

}

#endif
