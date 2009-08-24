
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>

#include "ocean.h"
#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "rosconsole/macros_generated.h"

using namespace std;
using namespace ros;
using namespace willowgarage::ocean;
namespace po = boost::program_options;

float toFloat(const int &value)
{
  int tmp = value;
  if(tmp & 0x8000)
    tmp = tmp - 65536;
  float result = tmp / 1000.0;
  return result;
}

int main(int argc, char** argv)
{
  string serial_device;
  int debug_level;
  int majorId = -1;     // Used for identity purposes
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "this help message")
    ("debug", po::value<int>(&debug_level)->default_value(0), "debug level")
    ("dev", po::value<string>(&serial_device)->default_value("/dev/ttyUSB0"), "serial device to open");

  po::variables_map vm;
  po::store(po::parse_command_line( argc, argv, desc), vm);
  po::notify(vm);

  if( vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  majorId = serial_device.at(serial_device.length() - 1) - '0';

  ros::init(argc, argv, "ocean_server");
  ros::NodeHandle handle;
  
  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  fprintf(stderr, "Logger Name: %s\n", ROSCONSOLE_DEFAULT_NAME);

  if( my_logger->getLevel() == 0 )    //has anyone set our level??
  {
    // Set the ROS logger
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
  }

  //
  //printf("device=%s  debug_level=%d\n", argv[1], atoi(argv[2]));
  //cout << "device=" << serial_device <<  "  debug_level=" << debug_level << endl;

  ocean os( debug_level);

  os.initialize(serial_device.c_str());

  ros::Publisher pub = handle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);

  ros::Rate rate(10);   //set the rate we scan the device for input
  diagnostic_msgs::DiagnosticArray msg_out;
  diagnostic_updater::DiagnosticStatusWrapper stat;
  Time lastTime, currentTime;
  std::stringstream ss;
  Duration MESSAGE_TIME(10,0);    //the message output rate

  lastTime = Time::now();

  while(handle.ok())
  {
    rate.sleep();
    ros::spinOnce();
    currentTime = Time::now();

    if((os.run() > 0) && ((currentTime - lastTime) > MESSAGE_TIME))
    {

      lastTime = currentTime;

      stat.values.clear();

      ss.str("");
      ss << "IBPS " << majorId;
      stat.name = ss.str();
      stat.level = 0;
      stat.message = "OK";
      
      stat.add("Time Remaining (min)", os.timeLeft);
      stat.add("Average charge (percent)", os.averageCharge );
      //stat.add("Current (A)", 0);
      //stat.add("Voltage (V)", 0);
      stat.add("Time since update (s)", (currentTime.sec - os.lastTimeSystem));

      msg_out.status.push_back(stat);

      for(unsigned int xx = 0; xx < os.MAX_BAT_COUNT; ++xx)
      {
        unsigned batmask = (1<<xx);
        if(os.present & batmask)
        {
          stat.values.clear();

          ss.str("");
          ss << "Smart Battery " << majorId << "." << xx;
          stat.name = ss.str();
          stat.level = 0;
          stat.message = "OK";
        
          stat.add("charging", (os.charging & batmask) ? "True":"False");
          stat.add("discharging", (os.discharging & batmask) ? "True":"False");
          stat.add("power present", (os.powerPresent & batmask) ? "True":"False");
          stat.add("No Good", (os.powerNG & batmask) ? "True":"False");
          stat.add("charge inhibited", (os.inhibited & batmask) ? "True":"False");

          for(unsigned int yy = 0; yy < os.regListLength; ++yy)
          {
            ss.str("");
            if(os.regList[yy].unit != "")
              ss << os.regList[yy].name << " (" << os.regList[yy].unit << ")";
            else
              ss << os.regList[yy].name;
            stat.add( ss.str(), os.batReg[xx][os.regList[yy].address]);
          }

          stat.add("Time since update (s)", (currentTime.sec - os.lastTimeBattery[xx]));

          msg_out.status.push_back(stat);
        }
      }

      pub.publish(msg_out);
    }
  }
}
