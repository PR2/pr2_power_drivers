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

#include "power_monitor.h"

using namespace std;

ros::PowerMonitor::PowerMonitor()
{
    ros::NodeHandle node;

    dynamic_reconfigure::Server<power_monitor::PowerMonitorConfig>::CallbackType config_callback = boost::bind(&ros::PowerMonitor::configCallback, this, _1, _2);
    config_server_.setCallback(config_callback);

    // Get the update frequency parameter
    double freq = 0.1;
    node.getParam("/power_monitor/frequency", freq);

    // Get the estimation method
    std::string estimator_type_str = "fuel gauge";
    node.getParam("/power_monitor/estimation_method", estimator_type_str);

    PowerStateEstimatorType e;
    if (!getPowerStateEstimatorType(estimator_type_str, e))
    {
        ROS_ERROR("Unknown power state estimator type: %s. Defaulting to fuel gauge", estimator_type_str.c_str());
        e = FuelGauge;
    }
    setPowerStateEstimator(e);

    // Publish to power_state
    pub_       = node.advertise<pr2_msgs::PowerState>("power_state", 5);
    pub_timer_ = node.createTimer(ros::Duration(1 / freq), &PowerMonitor::onPublishTimer, this);

    // Subscribe to battery/server
    sub_ = node.subscribe("battery/server", 10, &PowerMonitor::batteryServerUpdate, this);
}

void ros::PowerMonitor::configCallback(power_monitor::PowerMonitorConfig& config, uint32_t level)
{
    setPowerStateEstimator((PowerStateEstimatorType) config.power_state_estimator_);
}

bool ros::PowerMonitor::setPowerStateEstimator(PowerStateEstimatorType estimator_type)
{
    PowerStateEstimator* estimator;
    switch (estimator_type)
    {
        case FuelGauge: estimator = new FuelGaugePowerStateEstimator(); break;
        case Advanced:  estimator = new AdvancedPowerStateEstimator();  break;
        default:        return false;
    }

    power_state_estimator_ = boost::shared_ptr<PowerStateEstimator>(estimator);

    ROS_INFO("Setting power state estimation to %s", power_state_estimator_->getMethodName().c_str());

    return true;
}

bool ros::PowerMonitor::getPowerStateEstimatorType(const std::string& estimator_type_str, PowerStateEstimatorType& estimator_type)
{
    if (estimator_type_str == "fuel gauge")
        estimator_type = FuelGauge;
    else if (estimator_type_str == "advanced")
        estimator_type = Advanced;
    else
        return false;

    return true;
}

void ros::PowerMonitor::batteryServerUpdate(const boost::shared_ptr<const pr2_msgs::BatteryServer>& battery_server)
{
    boost::mutex::scoped_lock lock(battery_servers_mutex_);

    ROS_DEBUG("Received battery message: voltage=%.2f", toFloat(battery_server->battery[0].batReg[0x9]));

    battery_servers_[battery_server->id] = battery_server;
}

ros::PowerObservable ros::PowerMonitor::extractPowerObservable()
{
    boost::mutex::scoped_lock lock(battery_servers_mutex_);

    vector<ros::BatteryObservable> batteries;
    for (map<int, boost::shared_ptr<const pr2_msgs::BatteryServer> >::iterator i = battery_servers_.begin(); i != battery_servers_.end(); i++)
    {
        const pr2_msgs::BatteryServer* bs = i->second.get();

        bool ac_present = (bs->powerPresent == 0xF);    // all four batteries show AC present?

        for (unsigned int j = 0; j < bs->battery.size(); j++)
        {
            const pr2_msgs::BatteryState& b = bs->battery[j];

            float        voltage = toFloat(b.batReg[0x9]);
            float        current = toFloat(b.batReg[0xA]);
            unsigned int rsc     = b.batReg[0x0D];
            unsigned int tte     = b.batReg[0x12];
            unsigned int ttf     = b.batReg[0x13];

            batteries.push_back(BatteryObservable(ac_present, voltage, current, rsc, tte, ttf));

            ROS_DEBUG("Battery %d.%d: %6.2f V %6.2f A %6.2f W", bs->id, j + 1, voltage, current, current * voltage);
        }
    }

    return ros::PowerObservable(batteries);
}

void ros::PowerMonitor::onPublishTimer(const ros::TimerEvent& e)
{
    // Extract info from the battery server
    ros::PowerObservable observable = extractPowerObservable();
    ROS_DEBUG("Total power: %6.1f W", observable.getTotalPower());
    ROS_DEBUG("Min voltage: %6.2f V", observable.getMinVoltage());

    // Estimate time and capacity remaining
    ros::PowerStateEstimate estimate = power_state_estimator_->estimate(observable);
    ROS_DEBUG("Time remaining: %.0f min", estimate.time_remaining.toSec() / 60);
    ROS_DEBUG("Min capacity: %d%%",       estimate.relative_capacity);

    // Publish the estimate
    power_state_.header.stamp      = ros::Time::now();
    power_state_.AC_present        = observable.getAcCount();
    power_state_.power_consumption = observable.getTotalPower();
    power_state_.relative_capacity = (int8_t) estimate.relative_capacity;
    power_state_.prediction_method = "fuel gauge";
    power_state_.time_remaining    = estimate.time_remaining;

    pub_.publish(power_state_);
}

float ros::PowerMonitor::toFloat(int value)
{
    int tmp = value;
    if (tmp & 0x8000)
        tmp -= 65536;
    return tmp / 1000.0f;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "power_monitor");

    ros::PowerMonitor monitor;

    ros::spin();

    return 0;
}
