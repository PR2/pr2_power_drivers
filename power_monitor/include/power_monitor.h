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

#ifndef POWER_MONITOR_POWER_MONITOR_H
#define POWER_MONITOR_POWER_MONITOR_H

#include <stdlib.h>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "pr2_msgs/PowerState.h"
#include "pr2_msgs/BatteryServer.h"
#include "dynamic_reconfigure/server.h"

#include "power_monitor/PowerMonitorConfig.h"

#include "power_state_estimator.h"

namespace ros
{

/**
 * This class listens to BatteryServer2 messages and publishes PowerState messages with estimates of
 * the time remaining until the robot switches off (if discharging) or until the robot is fully charged
 * (if charging).
 */
class PowerMonitor
{
public:
    typedef enum { FuelGauge, Advanced } PowerStateEstimatorType;

    PowerMonitor();

    bool setPowerStateEstimator(PowerStateEstimatorType estimator_type);

    void batteryServerUpdate(const boost::shared_ptr<const pr2_msgs::BatteryServer>& bat);
    void configCallback(power_monitor::PowerMonitorConfig& config, uint32_t level);

private:
    ros::PowerObservable extractPowerObservable();

    bool getPowerStateEstimatorType(const std::string& estimator_type_str, PowerStateEstimatorType& estimator_type);

    void onPublishTimer(const ros::TimerEvent& e);

    static float toFloat(int value);

private:
    dynamic_reconfigure::Server<power_monitor::PowerMonitorConfig> config_server_;

    std::map<int, boost::shared_ptr<const pr2_msgs::BatteryServer> > battery_servers_;
    boost::mutex                                                     battery_servers_mutex_;

    ros::Timer      pub_timer_;
    ros::Publisher  pub_;
    ros::Subscriber sub_;

    ros::PowerObservable power_observable_;

    boost::shared_ptr<PowerStateEstimator> power_state_estimator_;

    pr2_msgs::PowerState power_state_;
};

}

#endif
