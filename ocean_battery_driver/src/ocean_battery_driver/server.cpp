
#include <iostream>
#include <vector>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>

#include "ocean.h"
#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "rosconsole/macros_generated.h"
#include "pr2_msgs/BatteryState.h"

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

class server
{
  public:

    server( const int &majorID, const std::string &dev, const int debug_level = 0 ) : 
      majorID(majorID), debug_level(debug_level), serial_device("/dev/ttyUSB0"), stopRequest(false)
    {
      std::stringstream ss;

      if(dev.empty())
      {
        string tmp_device;
        ss.str("");
        ss << "/dev/ttyUSB" << majorID; //create a default device based on majorID
        serial_device = ss.str();

        ss.str("");
        ss << "/ocean_server/port" << majorID;
        bool result = handle.getParam( ss.str(), tmp_device );
        if(result == true)
        {
          ROS_INFO("Using %s from getParam.\n", ss.str().c_str());
          serial_device = tmp_device;
        }
        else
          ROS_INFO("Defaulting to: %s", serial_device.c_str());

      }
      else
      {
        ROS_INFO("Overriding device with argument: %s", dev.c_str() );
        serial_device = dev;
      }

      //
      //printf("device=%s  debug_level=%d\n", argv[1], atoi(argv[2]));
      //cout << "device=" << serial_device <<  "  debug_level=" << debug_level << endl;

    }

    void start()
    {
      myThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&server::run, this)));
    }


    void stop()
    {
      stopRequest = true;
      myThread->join();
    }

  private:

    ros::NodeHandle handle;
    int majorID;
    int debug_level;
    std::string serial_device;
    volatile bool stopRequest;
    boost::shared_ptr<boost::thread> myThread;

    void run()
    {
      std::stringstream ss;

      //
      //  Need to make the queue size big enough that each thread can publish without
      //  concern that one message it quickly replaced by another threads message.
      //
      ros::Publisher pub    = handle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
      ros::Publisher bs     = handle.advertise<pr2_msgs::BatteryServer>("/battery/server", 10);

      ros::Rate rate(100);   //set the rate we scan the device for input
      diagnostic_msgs::DiagnosticArray msg_out;
      diagnostic_updater::DiagnosticStatusWrapper stat;
      Time lastTime, currentTime;
      Duration MESSAGE_TIME(2,0);    //the message output rate
      ocean os( majorID, debug_level);
      os.initialize(serial_device.c_str());

      lastTime = Time::now();

      while(handle.ok() && (stopRequest == false))
      {
        rate.sleep();
        //ros::spinOnce();
        currentTime = Time::now();

        if((os.run() > 0) && ((currentTime - lastTime) > MESSAGE_TIME))
        {

          // First publish our internal data
          bs.publish(os.server);

          lastTime = currentTime;

          stat.values.clear();

          ss.str("");
          ss << "IBPS " << majorID;
          stat.name = ss.str();
          stat.level = 0;
          stat.message = "OK";
          
          stat.add("Time Remaining (min)", os.server.timeLeft);
          stat.add("Average charge (percent)", os.server.averageCharge );
          stat.add("Time since update (s)", (currentTime.sec - os.server.lastTimeSystem));

          msg_out.status.push_back(stat);

          for(unsigned int xx = 0; xx < os.server.MAX_BAT_COUNT; ++xx)
          {
            unsigned batmask = (1<<xx);
            if(os.server.present & batmask)
            {
              stat.values.clear();

              ss.str("");
              ss << "Smart Battery " << majorID << "." << xx;
              stat.name = ss.str();
              stat.level = 0;
              stat.message = "OK";
            
              stat.add("charging", (os.server.charging & batmask) ? "True":"False");
              stat.add("discharging", (os.server.discharging & batmask) ? "True":"False");
              stat.add("power present", (os.server.powerPresent & batmask) ? "True":"False");
              stat.add("No Good", (os.server.powerNG & batmask) ? "True":"False");
              stat.add("charge inhibited", (os.server.inhibited & batmask) ? "True":"False");

              for(unsigned int yy = 0; yy < os.regListLength; ++yy)
              {
                unsigned addr = os.regList[yy].address;
                if(os.server.battery[xx].batRegFlag[addr] != 0)
                {
                  ss.str("");
                  if(os.regList[yy].unit != "")
                    ss << os.regList[yy].name << " (" << os.regList[yy].unit << ")";
                  else
                    ss << os.regList[yy].name;
                  stat.add( ss.str(), os.server.battery[xx].batReg[addr]);
                }
              }

              stat.add("Time since update (s)", (currentTime.sec - os.server.battery[xx].lastTimeBattery));

              msg_out.status.push_back(stat);

            }
          }

          pub.publish(msg_out);
          msg_out.status.clear();

        }
      }
    }

};

int main(int argc, char** argv)
{
  string tmp_device;
  int debug_level;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "this help message")
    ("debug", po::value<int>(&debug_level)->default_value(0), "debug level")
    ("dev", po::value<string>(&tmp_device), "serial device to open");

  po::variables_map vm;
  po::store(po::parse_command_line( argc, argv, desc), vm);
  po::notify(vm);

  if( vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  ros::init(argc, argv, "ocean_server");
  ros::NodeHandle handle;

  //majorID = serial_device.at(serial_device.length() - 1) - '0';

  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  fprintf(stderr, "Logger Name: %s\n", ROSCONSOLE_DEFAULT_NAME);

  if( my_logger->getLevel() == 0 )    //has anyone set our level??
  {
    // Set the ROS logger
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
  }

  int max_ports(4);
  handle.getParam( "/ocean_server/number_of_ports", max_ports );
  ROS_INFO("number_of_ports=%d", max_ports);
  handle.getParam( "/ocean_server/debug_level", debug_level );
  ROS_DEBUG("debug_level=%d", debug_level);

  vector<server> server_list;

  for(int xx = 0; xx < max_ports; ++xx)
    server_list.push_back(server( xx, tmp_device, debug_level));

  for(int xx = 0; xx < max_ports; ++xx)
    server_list[xx].start();

  ros::spin(); //wait for ros to shut us down


  for(int xx = 0; xx < max_ports; ++xx)
    server_list[xx].stop();

}
