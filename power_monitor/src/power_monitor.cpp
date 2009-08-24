
#include <stdlib.h>

#include "ros/ros.h"
#include "ros/node.h"
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
      ROS_INFO("Received battery message: voltage=%f", toFloat(bat->battery[0].batReg[0x9]));
    };

    void send()
    {
  #if 0
      float min_voltage = server_list[0].getVoltage(0);
      for(int xx = 1; xx < 4; ++xx)
      {
        if(server_list[0].getVoltage(xx) < min_voltage)
          min_voltage = server_list[0].getVoltage(xx);
      }
      cout << "min_voltage=" << min_voltage << endl;
  #endif

      powerState.power_consumption = 0;
      powerState.time_remaining = 60;
      powerState.prediction_method = "fuel guage";
      powerState.AC_present = 1;

      pub.publish(powerState);
    };

    PowerMonitor()
    { 
      //handle = new ros::NodeHandle();
      ros::NodeHandle handle;

      pub = handle.advertise<pr2_msgs::PowerState>("power_state", 5);
      sub = handle.subscribe("battery/server", 10, &PowerMonitor::batteryUpdate, this);
    };


  private:
    pr2_msgs::PowerState  powerState;
    ros::Publisher pub;
    ros::Subscriber sub;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "power_monitor");

  PowerMonitor monitor;
  ros::Rate rate(1);
  ros::NodeHandle handle;

  while(handle.ok())
  {
    rate.sleep();
    ros::spinOnce();


    monitor.send();
  }


}
