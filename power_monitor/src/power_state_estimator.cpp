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
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include "ros/ros.h"

using namespace std;
using namespace power_monitor;

// PowerStateEstimator

PowerStateEstimator::PowerStateEstimator() { }

void PowerStateEstimator::recordObservation(const PowerObservation& obs)
{
    obs_ = obs;
}

bool PowerStateEstimator::canEstimate(const ros::Time& t) const
{
    return t >= ros::Time::now() && obs_.getBatteries().size() > 0;
}

// FuelGaugePowerStateEstimator

string                    FuelGaugePowerStateEstimator::getMethodName() const { return "fuel gauge"; }
PowerStateEstimator::Type FuelGaugePowerStateEstimator::getMethodType() const { return FuelGauge;    }

PowerStateEstimate FuelGaugePowerStateEstimator::estimate(const ros::Time& t)
{
    PowerStateEstimate ps;
    ps.time_remaining    = obs_.getAcCount() > 0 ? obs_.getMaxTimeToFull(t) : obs_.getMinTimeToEmpty(t);
    ps.relative_capacity = obs_.getMinRelativeStateOfCharge();

    return ps;
}

// AdvancedPowerStateEstimator

const std::string AdvancedPowerStateEstimator::DEFAULT_LOG_FILE = "/hwlog/power_monitor/power.log";

AdvancedPowerStateEstimator::AdvancedPowerStateEstimator()
{
    ros::NodeHandle node;

    log_filename_ = DEFAULT_LOG_FILE;
    node.getParam("advanced_log_file", log_filename_);
    ROS_INFO("Using log file: %s", log_filename_.c_str());

    readObservations(log_);
    ROS_INFO("Read %d observations", (int) log_.size());
}

string                    AdvancedPowerStateEstimator::getMethodName() const { return "advanced"; }
PowerStateEstimator::Type AdvancedPowerStateEstimator::getMethodType() const { return Advanced;   }

void AdvancedPowerStateEstimator::recordObservation(const PowerObservation& obs)
{
    PowerStateEstimator::recordObservation(obs);

    // Ignore any observation with less than 16 batteries
    if (obs.getBatteries().size() != 16)
        return;

    LogRecord record;
    record.sec                          = obs.getStamp().sec;
    record.master_state                 = obs.getMasterState();
    record.charging                     = obs.getAcCount();
    record.total_power                  = obs.getTotalPower();
    record.min_voltage                  = obs.getMinVoltage();
    record.min_relative_state_of_charge = obs.getMinRelativeStateOfCharge();
    record.total_remaining_capacity     = obs.getTotalRemainingCapacity();
    log_.push_back(record);

    saveObservation(obs);
}

/**
 * Returns whether we've ever received a message that the power board is shutting down.
 * If we have, then we can use the remaining capacity reported at that point to bias our prediction.
 */
bool AdvancedPowerStateEstimator::hasEverDischarged() const
{
    for (vector<LogRecord>::const_iterator i = log_.begin(); i != log_.end(); i++)
        if ((*i).master_state == pr2_msgs::PowerBoardState::MASTER_SHUTDOWN)
            return true;

    return false;
}

PowerStateEstimate AdvancedPowerStateEstimator::estimate(const ros::Time& t)
{
    PowerStateEstimate ps;

    // If we have history of the batteries being completely drained, then offset our estimate by the minimum reported capacity
    if (log_.size() > 0 && obs_.getAcCount() == 0 && hasEverDischarged())
    {
        // Get the minimum remaining capacity reported ever
        unsigned int min_rsc     = 100;
        float        min_rem_cap = 999999.9;
        for (vector<LogRecord>::const_iterator i = log_.begin(); i != log_.end(); i++)
        {
	    if ((*i).master_state == pr2_msgs::PowerBoardState::MASTER_SHUTDOWN)
	    {
	        min_rsc     = min(min_rsc,     (*i).min_relative_state_of_charge);
	        min_rem_cap = min(min_rem_cap, (*i).total_remaining_capacity);
	    }
        }

        // @todo: should filter the noisy current
        float current        = obs_.getTotalPower() / obs_.getMinVoltage();

        float actual_rem_cap = obs_.getTotalRemainingCapacity() - min_rem_cap;
        float rem_hours      = actual_rem_cap / -current;

	ROS_DEBUG("current reported relative state of charge: %d", obs_.getMinRelativeStateOfCharge());
        ROS_DEBUG("minimum reported remaining capacity: %d", min_rem_cap);
        ROS_DEBUG("minimum reported relative state of charge: %d", min_rsc);
        ROS_DEBUG("current: %.2f", current);
        ROS_DEBUG("report remaining capacity: %f", obs_.getTotalRemainingCapacity());
        ROS_DEBUG("time remaining: %.2f mins", rem_hours * 60);

        ps.time_remaining = ros::Duration(rem_hours * 60 * 60);
	ps.relative_capacity = int(100 * (obs_.getMinRelativeStateOfCharge() - min_rsc) / (100.0 - min_rsc));
    }
    else
    {
        // No history. Resort to fuel gauge method
        ps.time_remaining = obs_.getAcCount() > 0 ? obs_.getMaxTimeToFull(t) : obs_.getMinTimeToEmpty(t);
	ps.relative_capacity = obs_.getMinRelativeStateOfCharge();
    }

    return ps;
}

void AdvancedPowerStateEstimator::tokenize(const string& str, vector<string>& tokens, const string& delimiters)
{
    string::size_type last_pos = str.find_first_not_of(delimiters, 0);
    string::size_type pos      = str.find_first_of(delimiters, last_pos);

    while (string::npos != pos || string::npos != last_pos)
    {
        tokens.push_back(str.substr(last_pos, pos - last_pos));

        last_pos = str.find_first_not_of(delimiters, pos);
        pos      = str.find_first_of(delimiters, last_pos);
    }
}

bool AdvancedPowerStateEstimator::logFileExists() const
{
    ifstream fin(log_filename_.c_str(), ios::in);
    bool exists = !fin.fail();
    if (exists)
        fin.close();

    return exists;
}

bool AdvancedPowerStateEstimator::readObservations(vector<LogRecord>& log)
{
    ifstream f(log_filename_.c_str(), ios::in);
    if (!f.is_open())
        return false;

    // Consume header line
    string line;
    getline(f, line);
    if (!f.good())
    {
        ROS_WARN("Error reading header from log file: %s", log_filename_.c_str());
        return false;
    }

    int line_num = 1;
    while (true)
    {
        getline(f, line);
        if (!f.good())
            break;

        vector<string> tokens;
        tokenize(line, tokens, ",");

        if (tokens.size() != 7)
        {
            ROS_WARN("Invalid line %d in log file: %s.  Aborting read.", line_num, log_filename_.c_str());
            break;
        }

        LogRecord record;
        record.sec                          = boost::lexical_cast<uint32_t>(tokens[0]);
        record.master_state                 = boost::lexical_cast<int>(tokens[1]);
        record.charging                     = boost::lexical_cast<int>(tokens[2]);
        record.total_power                  = boost::lexical_cast<float>(tokens[3]);
        record.min_voltage                  = boost::lexical_cast<float>(tokens[4]);
        record.min_relative_state_of_charge = boost::lexical_cast<unsigned int>(tokens[5]);
        record.total_remaining_capacity     = boost::lexical_cast<float>(tokens[6]);
        log.push_back(record);

        line_num++;
    }

    f.close();

    return true;
}

bool AdvancedPowerStateEstimator::saveObservation(const PowerObservation& obs) const
{
    bool exists = logFileExists();

    if (!exists)
    {
        // Check if the directory exists
        size_t last_dir_sep = log_filename_.rfind("/");
        if (last_dir_sep == string::npos)
        {
            ROS_ERROR("Invalid log file: %s", log_filename_.c_str());
            return false;
        }

        string log_dir = log_filename_.substr(0, last_dir_sep);

        struct stat s;
        if (stat(log_dir.c_str(), &s) != 0)
        {
            // Directory doesn't exist - create
            ROS_INFO("Creating log file directory: %s", log_dir.c_str());
            if (mkdir(log_dir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IWGRP | S_IXGRP | S_IROTH | S_IWOTH | S_IXOTH) != 0)      // create directory with permissions: rwxrwxrwx
            {
                ROS_ERROR("Error creating power monitor log file directory");
                return false;
            }
        }
    }

    // Open the log file for appending
    ofstream f(log_filename_.c_str(), ios::out | ios::app);
    if (f.fail())
    {
        ROS_ERROR("Error opening log file: %s", log_filename_.c_str());
        return false;
    }

    // Write the header if the file is new
    if (!exists)
        f << "secs,master_state,charging,total_power,min_voltage,min_relative_state_of_charge,total_remaining_capacity" << endl;

    // Append the observation row
    f << obs.getStamp().sec << ","
      << (int) obs.getMasterState() << ","
      << obs.getAcCount() << ","
      << obs.getTotalPower() << ","
      << obs.getMinVoltage() << ","
      << obs.getMinRelativeStateOfCharge() << ","
      << obs.getTotalRemainingCapacity() << endl;

    f.close();

    return true;
}
