
///\author Kevin Watts
///\brief Battery self test. Checks that all batteries responding

#include "ocean.h"

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <unistd.h>

#include <ros/time.h>
#include <ros/rate.h>

namespace po = boost::program_options;
using namespace std;
using namespace willowgarage::ocean;

class BatteryServerChecker
{
private:
  int id_;
  string device_;
  ocean os;
  bool stopRequest;

  vector<bool> present;
  vector<bool> charging;
  vector<bool> discharging;
  vector<bool> inhibited;

  boost::shared_ptr<boost::thread> runThread_;

  void run()
  {
    ros::Rate my_rate(10);

    while (!stopRequest)
    {
      os.run();
      for (int i = 0; i < os.server.MAX_BAT_COUNT; ++i)
      {
        present[i] = os.server.battery[i].present;
        charging[i] = os.server.battery[i].charging;
        discharging[i] = os.server.battery[i].discharging;
        inhibited[i] = os.server.battery[i].inhibited;
      }
        
      my_rate.sleep();
    }
  }

public:
  BatteryServerChecker(int id, const string &dev):
    id_(id), device_(dev), os(id, 0), stopRequest(false)
  {
    os.initialize(dev);

    present.resize(os.server.MAX_BAT_COUNT);
    charging.resize(os.server.MAX_BAT_COUNT);
    discharging.resize(os.server.MAX_BAT_COUNT);
    inhibited.resize(os.server.MAX_BAT_COUNT);
  }

  void start()
  {
    runThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&BatteryServerChecker::run, this)));
  }
      
  void stop()
  {
    stopRequest = true;
    runThread_->join();
  }

  bool batteryOK() const
  {
    bool ok = true;

    // Charge v discharging states should be same for all batteries
    bool charge_ok = true;
    bool discharge_ok = true;

    for (int i = 0; i < os.server.MAX_BAT_COUNT; ++i)
    {
      ok = ok && present[i]; // && !inhibited[i];
      charge_ok = charging[i] == charging[0] && charge_ok;
      discharge_ok = discharging[i] == discharging[0] && discharge_ok;
    }

    return ok && charge_ok && discharge_ok;
  }

  string getStatus() const
  {
    stringstream ss;
    ss.str("");
    for (int i = 0; i < os.server.MAX_BAT_COUNT; ++i)
      {
	ss << "\tBattery " << i << ":\n";
	ss << "\t\tPresent: " << (present[i] ? "OK" : "No") << "\n";
	ss << "\t\tCharging: " << (charging[i] ? "Yes" : "No") << "\n";
	ss << "\t\tDischarging: " << (discharging[i] ? "Yes" : "No") << "\n";
      }
    return ss.str();
  }
};

int main(int argc, char** argv)
{
  int duration;
  int min_duration;
  po::options_description desc("battery_check port1 port2 ... [options]");
  desc.add_options()
    ("help,h", "Print help message and exit")
    ("verbose,v", "Verbose battery output")
    ("duration,d", po::value<int>(&duration)->default_value(30), "Maximum duration of test")
    ("min-duration,m", po::value<int>(&min_duration)->default_value(5), "Minimum duration of test")
    ("port", po::value<vector<string> >(), "Battery ports to check");

  po::positional_options_description p;
  p.add("port", -1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
  po::notify(vm);

  bool verbose = vm.count("verbose");

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  if (!vm.count("port"))
  {
    fprintf(stderr, "No ports specified. Unable to check batteries.\n");
    return 1;
  }

  vector<string> ports(vm["port"].as< vector<string> >());
  vector<string>::iterator it;

  if (verbose)
  {
    for (it = ports.begin(); it != ports.end(); ++it)
      cout << "Checking port: " << *it << "\n";
  }

  if (verbose)
    cout << "Running battery check. Waiting for batteries to initialize\n";

  vector<boost::shared_ptr<BatteryServerChecker> > checkers;
  for (uint i = 0; i < ports.size(); ++i)
  {    
    checkers.push_back(boost::shared_ptr<BatteryServerChecker>(new BatteryServerChecker(i, ports[i])));
    checkers[i]->start();                       
  }


  ros::Rate my_rate(2);
  ros::Time startTime = ros::Time::now();

  ros::Duration min(min_duration);
  ros::Duration max(duration);

  bool all_ok = false;
  while (true)
  {
    my_rate.sleep();
    
    if (ros::Time::now() - startTime < min)
      continue;

    if (ros::Time::now() - startTime > max)
      break;

    bool now_ok = true;
    for (uint i = 0; i < checkers.size(); ++i)
      now_ok = checkers[i]->batteryOK() && now_ok;

    if (now_ok)
    {
      all_ok = true;
      break;
    }
  }
  
  for (uint i = 0; i < checkers.size(); ++i)
    checkers[i]->stop();

  if (all_ok)
  {
    fprintf(stdout, "All batteries OK\n");
    return 0;
  }

  fprintf(stderr, "Not all batteries reported.\n");
  cout << "Status: \n";
  for (uint i = 0; i < checkers.size(); ++i)
    cout << "\tBattery " << i << ": " << (checkers[i]->batteryOK() ? string("OK") : string("No report")) << "\n";
  
  if (verbose)
    {
      fprintf(stderr, "Battery error reports:\n");
      for (uint i = 0; i < checkers.size(); ++i)
	cerr << checkers[i]->getStatus().c_str();
    }

  return 1;
}
