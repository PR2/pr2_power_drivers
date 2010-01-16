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
using namespace power_monitor;

PowerMonitor::PowerMonitor()
{
    ros::NodeHandle node;

    ros::Duration(2).sleep();

    // Register the estimators
    addEstimator(new FuelGaugePowerStateEstimator());
    addEstimator(new AdvancedPowerStateEstimator());

    // Setup the dynamic_reconfigure callback
    dynamic_reconfigure::Server<PowerMonitorConfig>::CallbackType config_callback = boost::bind(&PowerMonitor::configCallback, this, _1, _2);
    config_server_.setCallback(config_callback);

    // Set the active estimation method
    string estimator_type_str = "fuel gauge";
    node.getParam("/power_monitor/estimation_method", estimator_type_str);

    if (estimator_types_.size() == 0)
    {
        ROS_FATAL("No power state estimators defined");
        exit(1);
    }
    map<string, PowerStateEstimator::Type>::const_iterator i = estimator_types_.find(estimator_type_str);
    if (i == estimator_types_.end())
    {
        // Request estimator is unknown. Default to first estimator
        string first_estimator_type_str = estimator_types_.begin()->first;
        ROS_ERROR("Unknown power state estimator type: %s. Defaulting to %s", estimator_type_str.c_str(), first_estimator_type_str.c_str());
        setActiveEstimator(estimator_types_.begin()->second);
    }
    else
        setActiveEstimator(i->second);

    // Publish to power_state
    pub_ = node.advertise<pr2_msgs::PowerState>("power_state", 5);

    // Create timer for publishing
    double freq = 0.1;
    node.getParam("/power_monitor/frequency", freq);
    pub_timer_ = node.createTimer(ros::Duration(1.0 / freq), &PowerMonitor::onPublishTimer, this);

    // Subscribe to battery/server
    sub_ = node.subscribe("battery/server", 10, &PowerMonitor::batteryServerUpdate, this);
}

void PowerMonitor::addEstimator(PowerStateEstimator* est)
{
    estimator_types_[est->getMethodName()] = est->getMethodType();
    estimators_[est->getMethodType()] = boost::shared_ptr<PowerStateEstimator>(est);
}

void PowerMonitor::configCallback(PowerMonitorConfig& config, uint32_t level)
{
    setActiveEstimator((PowerStateEstimator::Type) config.power_state_estimator_);
}

bool PowerMonitor::setActiveEstimator(PowerStateEstimator::Type estimator_type)
{
    map<PowerStateEstimator::Type, boost::shared_ptr<PowerStateEstimator> >::const_iterator i = estimators_.find(estimator_type);
    if (i == estimators_.end())
        return false;
    if (active_estimator_ == i->second)
        return true;

    active_estimator_ = i->second;

    ROS_INFO("Power state estimator set to %s", active_estimator_->getMethodName().c_str());

    return true;
}

void PowerMonitor::batteryServerUpdate(const boost::shared_ptr<const pr2_msgs::BatteryServer>& battery_server)
{
    boost::mutex::scoped_lock lock(battery_servers_mutex_);

    ROS_DEBUG("Received battery message: voltage=%.2f", toFloat(battery_server->battery[0].batReg[0x9]));

    battery_servers_[battery_server->id] = battery_server;
}

PowerObservation PowerMonitor::extractObservation()
{
    boost::mutex::scoped_lock lock(battery_servers_mutex_);

    vector<BatteryObservation> batteries;
    for (map<int, boost::shared_ptr<const pr2_msgs::BatteryServer> >::iterator i = battery_servers_.begin(); i != battery_servers_.end(); i++)
    {
        const pr2_msgs::BatteryServer* bs = i->second.get();

        ros::Time stamp      = bs->header.stamp;
        bool      ac_present = (bs->powerPresent == 0xF);    // all four batteries show AC present?

        for (unsigned int j = 0; j < bs->battery.size(); j++)
        {
            const pr2_msgs::BatteryState& b = bs->battery[j];

            float        voltage = toFloat(b.batReg[0x9]);
            float        current = toFloat(b.batReg[0xA]);
            unsigned int rsc     = b.batReg[0x0D];
            float        rem_cap = b.batReg[0x0F] / 1000.0f;
            unsigned int tte_min = b.batReg[0x12];
            unsigned int ttf_min = b.batReg[0x13];

            ros::Duration tte(tte_min * 60, 0);
            ros::Duration ttf(ttf_min * 60, 0);

            batteries.push_back(BatteryObservation(stamp, ac_present, voltage, current, rsc, rem_cap, tte, ttf));

            ROS_DEBUG("Battery %d.%d: %6.2f V %6.2f A %6.2f W", bs->id, j + 1, voltage, current, current * voltage);
        }
    }

    return PowerObservation(ros::Time::now(), batteries);
}

void PowerMonitor::onPublishTimer(const ros::TimerEvent& e)
{
    // Extract info from the battery server
    PowerObservation obs = extractObservation();
    ROS_DEBUG("Total power: %6.1f W", obs.getTotalPower());
    ROS_DEBUG("Min voltage: %6.2f V", obs.getMinVoltage());

    // Give every estimator the chance to record this observation
    for (map<PowerStateEstimator::Type, boost::shared_ptr<PowerStateEstimator> >::const_iterator i = estimators_.begin(); i != estimators_.end(); i++)
        i->second.get()->recordObservation(obs);

    // Use the active estimator to estimate the time and capacity remaining
    ros::Time t = ros::Time::now();
    if (active_estimator_->canEstimate(t))
    {
        PowerStateEstimate estimate = active_estimator_->estimate(t);
        ROS_DEBUG("Time remaining: %.0f min", estimate.time_remaining.toSec() / 60);
        ROS_DEBUG("Min capacity: %d%%",       estimate.relative_capacity);

        // Publish the power state estimate
        pr2_msgs::PowerState ps;
        ps.header.stamp      = ros::Time::now();
        ps.AC_present        = obs.getAcCount();
        ps.power_consumption = obs.getTotalPower();
        ps.prediction_method = active_estimator_->getMethodName();
        ps.relative_capacity = (int8_t) estimate.relative_capacity;
        ps.time_remaining    = estimate.time_remaining;
        pub_.publish(ps);
    }
}

float PowerMonitor::toFloat(int value)
{
    int tmp = value;
    if (tmp & 0x8000)
        tmp -= 65536;
    return tmp / 1000.0f;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "power_monitor");
    PowerMonitor monitor;
    ros::spin();
    return 0;
}
