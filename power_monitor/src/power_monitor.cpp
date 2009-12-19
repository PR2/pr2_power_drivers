
#include <stdlib.h>
#include <boost/thread/mutex.hpp>

#include "ros/ros.h"
#include "pr2_msgs/PowerState.h"
#include "pr2_msgs/BatteryServer.h"

using namespace std;
using namespace ros;

static float toFloat(const int &value)
{
  int tmp = value;
  if(tmp & 0x8000)
    tmp = tmp - 65536;
  float result = tmp / 1000.0;
  return result;
}

class PowerMonitor
{
  public:

    void batteryUpdate( const boost::shared_ptr<pr2_msgs::BatteryServer const> &bat)
    {
      boost::mutex::scoped_lock lock(vLock);
      ROS_DEBUG("Received battery message: voltage=%f", toFloat(bat->battery[0].batReg[0x9]));

      batteryServers[bat->id] = bat;    //add results to our map
    };

    void send(const ros::TimerEvent &e)
    {
      float min_voltage(1000.0);
      int acCount(0);
      //float totalCurrent[batteryServers.size()];
      float totalPower(0.0);
      unsigned int minTime(65534);  //for discharging
      unsigned int maxTime(0);  //for charging
      unsigned int minCapacity(1000); 

      boost::mutex::scoped_lock lock(vLock);
      map< int, boost::shared_ptr<pr2_msgs::BatteryServer const> >::iterator itr = batteryServers.begin();
      for( ; itr != batteryServers.end(); ++itr )
      {
        const pr2_msgs::BatteryServer *bat = itr->second.get();

        ROS_DEBUG("------------------------------------");
        ROS_DEBUG("BATTERY  %d", bat->id);

        if(bat->powerPresent  == 0xF)  //All four batteries show AC present
          ++acCount;


        for(unsigned int xx = 0; xx < bat->MAX_BAT_COUNT; ++xx)
        {
          float voltage = toFloat(bat->battery[xx].batReg[0x9]);
          ROS_DEBUG(" voltage=%f", voltage);
          if(voltage < min_voltage)
            min_voltage = voltage;

          float current = toFloat(bat->battery[xx].batReg[0xA]);
          ROS_DEBUG(" current=%f", current);
          ROS_DEBUG(" power=%f", current * voltage);
          totalPower += (current * voltage);

          unsigned tte = bat->battery[xx].batReg[0x12];
          if((tte != 65535) && (tte < minTime)) //search for battery with least time remaining
            minTime = tte;

          unsigned ttf = bat->battery[xx].batReg[0x13];
          if((ttf != 65535) && (ttf > maxTime)) //search for battery with longtest recharge time
            maxTime = ttf;

          unsigned rsc = bat->battery[xx].batReg[0x0d];
          if(rsc < minCapacity) //search for battery with lowest state of charge
            minCapacity = rsc;
        }

        //totalCurrent[bat->id] = tmpCurrent;
        //cout << " totalCurrent=" << tmpCurrent << "\n";
      }
      //ROS_DEBUG("PowerMonitor::min_voltage=%f", min_voltage);
      if(minCapacity == 1000)
        minCapacity = 0;

      ROS_DEBUG("PowerMonitor::minCapacity=%d", minCapacity);
      ROS_DEBUG(" totalPower=%f", totalPower);

      powerState.power_consumption = totalPower;
      powerState.AC_present = acCount;
      if(acCount > 0) //Are we charging??
        powerState.time_remaining.fromSec(maxTime * 60);  //convert minutes to seconds
      else
        powerState.time_remaining.fromSec((int)minTime * 60);  //convert minutes to seconds
      powerState.prediction_method = "fuel gauge";
      powerState.relative_capacity = (int8_t)minCapacity;

      // FIX ME: Setting time to ros::Time::now() before publishing.
      // This may not be correct, but at least gets rid of deprecation
      // warning.
      powerState.header.stamp = ros::Time::now();
      pub.publish(powerState);
    };


    PowerMonitor()
    { 
      //handle = new ros::NodeHandle();
      ros::NodeHandle handle;
      double freq = 0.1;
      handle.getParam("/power_monitor/frequency", freq);

      powerState.power_consumption = 0;
      powerState.time_remaining.fromSec(0);
      powerState.relative_capacity = 0;
      powerState.AC_present = 0;

      pub = handle.advertise<pr2_msgs::PowerState>("power_state", 5);
      sub = handle.subscribe("battery/server", 10, &PowerMonitor::batteryUpdate, this);

      timer = handle.createTimer(ros::Duration(1/freq), &PowerMonitor::send, this);
    };


  private:
    pr2_msgs::PowerState  powerState;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Timer timer;
    boost::mutex vLock;
    std::map< int, boost::shared_ptr<pr2_msgs::BatteryServer const> > batteryServers;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "power_monitor");

  PowerMonitor monitor;

  ros::spin();

  return 0;
}
