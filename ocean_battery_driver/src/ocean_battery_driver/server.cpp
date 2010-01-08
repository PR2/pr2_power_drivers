
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
#include "pr2_msgs/BatteryServer.h" //This is here to send a copy of the previous message

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

      ros::NodeHandle private_handle("~");

      if(dev.empty())
      {
        string tmp_device;
        ss.str("");
        ss << "/dev/ttyUSB" << majorID; //create a default device based on majorID
        serial_device = ss.str();

        ss.str("");
        ss << "port" << majorID;
        bool result = private_handle.getParam( ss.str(), tmp_device );
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
      ros::Publisher bs2    = handle.advertise<pr2_msgs::BatteryServer2>("/battery/server2", 10);
      ros::Publisher bs     = handle.advertise<pr2_msgs::BatteryServer>("/battery/server", 10);

      ros::Rate rate(100);   //set the rate we scan the device for input
      diagnostic_msgs::DiagnosticArray msg_out;
      diagnostic_updater::DiagnosticStatusWrapper stat;
      Time lastTime, currentTime;
      Duration MESSAGE_TIME(2,0);    //the message output rate
      ocean os( majorID, debug_level);
      os.initialize(serial_device.c_str());

      pr2_msgs::BatteryServer oldserver;
      oldserver.battery.resize(4);

      lastTime = Time::now();

      while(handle.ok() && (stopRequest == false))
      {
        rate.sleep();
        //ros::spinOnce();
        currentTime = Time::now();

        if((os.run() > 0) && ((currentTime - lastTime) > MESSAGE_TIME))
        {

          // First publish our internal data

          // FIX ME:
          // Setting time to currentTime before publishing.  This may
          // not be correct, but at least gets rid of deprecationg
          // warning.
          os.server.header.stamp = currentTime;
          bs2.publish(os.server);

          oldserver.id = os.server.id;
          oldserver.lastTimeSystem = os.server.last_system_update.sec;
          oldserver.timeLeft = os.server.time_left.toSec()/60;
          oldserver.averageCharge = os.server.average_charge;
          oldserver.message = os.server.message;
          oldserver.lastTimeController = os.server.last_controller_update.sec;
          oldserver.present = os.server.battery[0].present * 1 + os.server.battery[1].present * 2 + os.server.battery[2].present * 4 + os.server.battery[3].present * 8;
          oldserver.charging = os.server.battery[0].charging * 1 + os.server.battery[1].charging * 2 + os.server.battery[2].charging * 4 + os.server.battery[3].charging * 8;
          oldserver.discharging = os.server.battery[0].discharging * 1 + os.server.battery[1].discharging * 2 + os.server.battery[2].discharging * 4 + os.server.battery[3].discharging * 8;
          //oldserver.reserved = os.server.battery[0].reserved * 1 + os.server.battery[1].reserved * 2 + os.server.battery[2].reserved * 4 + os.server.battery[3].reserved * 8;
          oldserver.powerPresent = os.server.battery[0].power_present * 1 + os.server.battery[1].power_present * 2 + os.server.battery[2].power_present * 4 + os.server.battery[3].power_present * 8;
          oldserver.powerNG = os.server.battery[0].power_no_good * 1 + os.server.battery[1].power_no_good * 2 + os.server.battery[2].power_no_good * 4 + os.server.battery[3].power_no_good * 8;
          oldserver.inhibited = os.server.battery[0].inhibited * 1 + os.server.battery[1].inhibited * 2 + os.server.battery[2].inhibited * 4 + os.server.battery[3].inhibited * 8;

          for(int xx = 0; xx < os.server.MAX_BAT_COUNT; ++xx)
          {
            oldserver.battery[xx].lastTimeBattery = os.server.battery[xx].last_battery_update.sec;
            for(unsigned int yy = 0; yy < os.regListLength; ++yy)
            {
              oldserver.battery[xx].batReg[yy] = os.server.battery[xx].battery_register[yy];
              oldserver.battery[xx].batRegFlag[yy] = os.server.battery[xx].battery_update_flag[yy];
              oldserver.battery[xx].batRegTime[yy] = os.server.battery[xx].battery_register_update[yy].sec;
            }
          }

          // FIX ME:
          // Setting time to currentTime before publishing.  This may
          // not be correct, but at least gets rid of deprecationg
          // warning.
          oldserver.header.stamp = currentTime;
          bs.publish(oldserver);

          lastTime = currentTime;

          stat.values.clear();

          ss.str("");
          ss << "IBPS " << majorID;
          stat.name = ss.str();
          stat.level = 0;
          stat.message = "OK";
          
          stat.add("Time Remaining (min)", (os.server.time_left.toSec()/60));
          stat.add("Average charge (percent)", os.server.average_charge );
          Duration elapsed = currentTime - os.server.last_system_update;
          stat.add("Time since update (s)", elapsed.toSec());

          // FIX ME:
          // Setting time to currentTime before publishing.  This may
          // not be correct, but at least gets rid of deprecationg
          // warning.
          msg_out.header.stamp = currentTime;
          msg_out.status.push_back(stat);

          for(int xx = 0; xx < os.server.MAX_BAT_COUNT; ++xx)
          {
            if(os.server.battery[xx].present)
            {
              stat.values.clear();

              ss.str("");
              ss << "Smart Battery " << majorID << "." << xx;
              stat.name = ss.str();
              stat.level = 0;
              stat.message = "OK";
            
              stat.add("charging", (os.server.battery[xx].charging) ? "True":"False");
              stat.add("discharging", (os.server.battery[xx].discharging) ? "True":"False");
              stat.add("power present", (os.server.battery[xx].power_present) ? "True":"False");
              stat.add("No Good", (os.server.battery[xx].power_no_good) ? "True":"False");
              stat.add("charge inhibited", (os.server.battery[xx].inhibited) ? "True":"False");

              for(unsigned int yy = 0; yy < os.regListLength; ++yy)
              {
                unsigned addr = os.regList[yy].address;
                if(os.server.battery[xx].battery_update_flag[addr])
                {
                  ss.str("");
                  if(os.regList[yy].unit != "")
                    ss << os.regList[yy].name << " (" << os.regList[yy].unit << ")";
                  else
                    ss << os.regList[yy].name;
                  
                  if(addr == 0x1b)
                  {
                    std::stringstream date;
                    date.str("");

                    unsigned int day = os.server.battery[xx].battery_register[addr] & 0x1F;
                    unsigned int month = (os.server.battery[xx].battery_register[addr] >> 5) & 0xF;
                    unsigned int year = (os.server.battery[xx].battery_register[addr] >> 9) + 1980;
                    date << day << "/" << month << "/" << year;
                    
                    stat.add( ss.str(), date.str());
                  }
                  else
                    stat.add( ss.str(), os.server.battery[xx].battery_register[addr]);
                }
              }

              elapsed = currentTime - os.server.battery[xx].last_battery_update;
              stat.add("Time since update (s)", elapsed.toSec());

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
  int max_ports;

  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "this help message")
    ("debug", po::value<int>(&debug_level)->default_value(0), "debug level")
    ("count", po::value<int>(&max_ports)->default_value(4), "number of ports to monitor")
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
  ros::NodeHandle private_handle("~");

  //majorID = serial_device.at(serial_device.length() - 1) - '0';

  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  fprintf(stderr, "Logger Name: %s\n", ROSCONSOLE_DEFAULT_NAME);

  if( my_logger->getLevel() == 0 )    //has anyone set our level??
  {
    // Set the ROS logger
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
  }

  private_handle.getParam( "number_of_ports", max_ports );
  ROS_INFO("number_of_ports=%d", max_ports);
  private_handle.getParam( "debug_level", debug_level );
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
