/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/select.h>
#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <signal.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/program_options.hpp>

// Internet/Socket stuff
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>

#include "power_comm.h"
#include "power_node.h"
#include "diagnostic_msgs/DiagnosticMessage.h"
#include "rosconsole/macros_generated.h"
#include "ros/ros.h"

using namespace std;
namespace po = boost::program_options;

// Keep a pointer to the last message recieved for
// Each board.
static std::vector<Device*> Devices;
static std::vector<Interface*> SendInterfaces;
static PowerBoard *myBoard;
static Interface* ReceiveInterface;

static const ros::Duration TIMEOUT = ros::Duration(1,0);


void Device::setTransitionMessage(const TransitionMessage &newtmsg)
{
  if (tmsgset)
  {
#define PRINT_IF_CHANGED(val)                                                                                 \
for (int counter = 0; counter < 3; counter++)                                                                 \
{                                                                                                             \
  if (tmsg.cb[counter].val##_count != newtmsg.cb[counter].val##_count)                                        \
  {                                                                                                           \
    ROS_INFO("Power board: CB%i "#val" event count changed to %i.", counter, newtmsg.cb[counter].val##_count);\
  }                                                                                                           \
}

  PRINT_IF_CHANGED(stop);
  PRINT_IF_CHANGED(estop);
  PRINT_IF_CHANGED(trip);
  PRINT_IF_CHANGED(fail_18V);
  PRINT_IF_CHANGED(disable);
  PRINT_IF_CHANGED(start);
  PRINT_IF_CHANGED(pump_fail);
  PRINT_IF_CHANGED(reset);

#undef PRINT_IF_CHANGED
  }
  else
    tmsgset = 1;

  tmsg = newtmsg;
}

void Device::setPowerMessage(const PowerMessage &newpmsg)
{
  if (pmsgset)
  {
#define PRINT_IF_CHANGED(val)                                                            \
if (pmsg.status.val##_status != newpmsg.status.val##_status)                             \
{                                                                                        \
  ROS_INFO("Power board: status of "#val" changed to %i.", newpmsg.status.val##_status); \
} 

  PRINT_IF_CHANGED(CB0);  
  PRINT_IF_CHANGED(CB1);
  PRINT_IF_CHANGED(CB2);
  PRINT_IF_CHANGED(estop_button);
  PRINT_IF_CHANGED(estop);

#undef PRINT_IF_CHANGED
  }
  else
    pmsgset = 1;

  pmsg = newpmsg;
}

Interface::Interface(const char* ifname) :
  recv_sock(-1),
  send_sock(-1) {
  memset(&interface, 0, sizeof(interface));
  assert(strlen(ifname) <= sizeof(interface.ifr_name));
  strncpy(interface.ifr_name, ifname, IFNAMSIZ);
}


void Interface::Close() {
  if (recv_sock != -1) {
    close(recv_sock);
    recv_sock = -1;
  }
  if (send_sock != -1) {
    close(send_sock);
    send_sock = -1;
  }
}


int Interface::InitReceive()
{

  recv_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (recv_sock == -1) {
    perror("Couldn't create recv socket");
    return -1;
  }

 // Allow reuse of recieve port
  int opt = 1;
  if (setsockopt(recv_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("Couldn't set reuse addr on recv socket\n");
    Close();
    return -1;
  }

  // Allow broadcast on send socket
  opt = 1;
  if (setsockopt(recv_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt))) {
    perror("Setting broadcast option on recv");
    Close();
    return -1;
  }

  // Bind socket to recieve packets on <UDP_STATUS_PORT> from any address/interface
  struct sockaddr_in sin;
  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_port = htons(POWER_PORT);
  sin.sin_addr.s_addr = (INADDR_ANY);
  if (bind(recv_sock, (struct sockaddr*)&sin, sizeof(sin))) {
    perror("Couldn't Bind socket to port");
    Close();
    return -1;
  }

  return 0;
}

int Interface::Init(sockaddr_in *port_address, sockaddr_in *broadcast_address)
{

  memcpy( &ifc_address, broadcast_address, sizeof(sockaddr_in));
#if 0
  recv_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (recv_sock == -1) {
    perror("Couldn't create recv socket");
    return -1;
  }
#endif
  send_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (send_sock == -1) {
    Close();
    perror("Couldn't create send socket");
    return -1;
  }


 // Allow reuse of recieve port
  int opt;
#if 0
  opt = 1;
  if (setsockopt(recv_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("Couldn't set reuse addr on recv socket\n");
    Close();
    return -1;
  }

  // Allow broadcast on send socket
  opt = 1;
  if (setsockopt(recv_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt))) {
    perror("Setting broadcast option on recv");
    Close();
    return -1;
  }
#endif
  // All recieving packets sent to broadcast address
  opt = 1;
  if (setsockopt(send_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt))) {
    perror("Setting broadcast option on send");
    Close();
    return -1;
  }

  // Bind socket to recieve packets on <UDP_STATUS_PORT> from any address/interface
  struct sockaddr_in sin;
  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
#if 0
  sin.sin_port = htons(POWER_PORT);
  sin.sin_addr.s_addr = (INADDR_ANY);
  //sin.sin_addr= port_address->sin_addr;
  if (bind(recv_sock, (struct sockaddr*)&sin, sizeof(sin))) {
    perror("Couldn't Bind socket to port");
    Close();
    return -1;
  }
#endif

  // Connect send socket to use broadcast address and same port as recieve sock
  sin.sin_port = htons(POWER_PORT);
  //sin.sin_addr.s_addr = INADDR_BROADCAST; //inet_addr("192.168.10.255");
  sin.sin_addr= broadcast_address->sin_addr;
  if (connect(send_sock, (struct sockaddr*)&sin, sizeof(sin))) {
    perror("Connect'ing socket failed");
    Close();
    return -1;
  }

  return 0;
}

void Interface::AddToReadSet(fd_set &set, int &max_sock) const {
  FD_SET(recv_sock,&set);
  if (recv_sock > max_sock)
    max_sock = recv_sock;
}

bool Interface::IsReadSet(fd_set set) const {
  return FD_ISSET(recv_sock,&set);
}

int PowerBoard::send_command(unsigned int serial_number, int circuit_breaker, const std::string &command, unsigned flags)
{
  if (Devices.size() == 0) {
    fprintf(stderr,"No devices to send command to\n");
    return -1;
  }

  int selected_device = -1;
  // Look for device serial number in list of devices...
  for (unsigned i = 0; i<Devices.size(); ++i) {
    if (Devices[i]->getPowerMessage().header.serial_num == serial_number) {
      selected_device = i;
      break;
    }
  }

  if ((selected_device < 0) || (selected_device >= (int)Devices.size())) {
    fprintf(stderr, "Device number must be between 0 and %u\n", Devices.size()-1);
    return -1;
  }

  Device* device = Devices[selected_device];
  assert(device != NULL);

  if ((circuit_breaker < 0) || (circuit_breaker > 2)) {
    fprintf(stderr, "Circuit breaker number must be between 0 and 2\n");
    return -1;
  }

  ROS_DEBUG("circuit=%d command=%s flags=%x\n", circuit_breaker, command.c_str(), flags);

  // Determine what command to send
  char command_enum = NONE;
  if (command == "start") {
    command_enum = COMMAND_START;
  }
  else if (command ==  "stop") {
    command_enum = COMMAND_STOP;
  }
  else if (command == "reset") {
    command_enum = COMMAND_RESET;
  }
  else if (command == "disable") {
    command_enum = COMMAND_DISABLE;
  }
  else if (command == "none") {
    command_enum = NONE;
  }
  else {
    ROS_ERROR("invalid command '%s'", command.c_str());
    return -1;
  }
  //" -c : command to send to device : 'start', 'stop', 'reset', 'disable'\n"


  // Build command message
  CommandMessage cmdmsg;
  memset(&cmdmsg, 0, sizeof(cmdmsg));
  cmdmsg.header.message_revision = CURRENT_MESSAGE_REVISION;
  cmdmsg.header.message_id = MESSAGE_ID_COMMAND;
  cmdmsg.header.serial_num = device->getPowerMessage().header.serial_num;
  //cmdmsg.header.serial_num = 0x12345678;
  strncpy(cmdmsg.header.text, "power command message", sizeof(cmdmsg.header.text));
  cmdmsg.command.CB0_command = NONE;
  cmdmsg.command.CB1_command = NONE;
  cmdmsg.command.CB2_command = NONE;
  if (circuit_breaker==0) {
    cmdmsg.command.CB0_command = command_enum;
  }
  else if (circuit_breaker==1) {
    cmdmsg.command.CB1_command = command_enum;
  }
  else if (circuit_breaker==2) {
    cmdmsg.command.CB2_command = command_enum;
  }
  else if (circuit_breaker==-1) {
    cmdmsg.command.CB0_command = command_enum;
    cmdmsg.command.CB1_command = command_enum;
    cmdmsg.command.CB2_command = command_enum;
  }

  cmdmsg.command.flags = flags;

  errno = 0;
  for (unsigned xx = 0; xx < SendInterfaces.size(); ++xx)
  {
    //ROS_INFO("Send on %s", inet_ntoa(SendInterfaces[xx]->ifc_address.sin_addr));
    int result = send(SendInterfaces[xx]->send_sock, &cmdmsg, sizeof(cmdmsg), 0);
    if (result == -1) {
      ROS_ERROR("Error sending");
      return -1;
    } else if (result != sizeof(cmdmsg)) {
      ROS_WARN("Error sending : send only took %d of %d bytes\n",
              result, sizeof(cmdmsg));
    }
  }

  ROS_DEBUG("Send to Serial=%u, revision=%u", cmdmsg.header.serial_num, cmdmsg.header.message_revision);
  ROS_DEBUG("Sent command %s(%d) to device %d, circuit %d",
         command.c_str(), command_enum, selected_device, circuit_breaker);

  return 0;
}


const char* PowerBoard::cb_state_to_str(char state)
{
  //enum CB_State { STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED };
  switch(state) {
  case STATE_NOPOWER:
    return "no-power";
  case STATE_STANDBY:
    return "Standby";
  case STATE_PUMPING:
    return "pumping";
  case STATE_ON:
    return "On";
  case STATE_DISABLED:
    return "Disabled";
  }
  return "???";
}

const char* PowerBoard::master_state_to_str(char state)
{
  //enum CB_State { STATE_NOPOWER, STATE_STANDBY, STATE_PUMPING, STATE_ON, STATE_DISABLED };
  switch(state) {
  case MASTER_NOPOWER:
    return "no-power";
  case MASTER_STANDBY:
    return "stand-by";
  case MASTER_ON:
    return "on";
  case MASTER_OFF:
    return "off";
  }
  return "???";
}


// Determine if a record of the device already exists...
// If it does use newest message a fill in pointer to old one .
// If it does not.. use
int PowerBoard::process_message(const PowerMessage *msg)
{
  if (msg->header.message_revision != CURRENT_MESSAGE_REVISION) {
    ROS_WARN("Got message with incorrect message revision %u\n", msg->header.message_revision);
    return -1;
  }

  // Look for device serial number in list of devices...
  if(serial_number != 0)  // when a specific serial is called out, ignore everything else
  {
    if(serial_number == msg->header.serial_num) //this should be our number
    {
      boost::mutex::scoped_lock(library_lock_);
      Devices[0]->message_time = ros::Time::now();
      Devices[0]->setPowerMessage(*msg);
    }
  }
  else
  {
    for (unsigned i = 0; i<Devices.size(); ++i) {
      if (Devices[i]->getPowerMessage().header.serial_num == msg->header.serial_num) {
        boost::mutex::scoped_lock(library_lock_);
        Devices[i]->message_time = ros::Time::now();
        Devices[i]->setPowerMessage(*msg);
        return 0;
      }
    }

    // Add new device to list
    Device *newDevice = new Device();
    Devices.push_back(newDevice);
    newDevice->message_time = ros::Time::now();
    newDevice->setPowerMessage(*msg);
  }

  return 0;
}

int PowerBoard::process_transition_message(const TransitionMessage *msg)
{
  if (msg->header.message_revision != CURRENT_MESSAGE_REVISION) {
    ROS_WARN("Got message with incorrect message revision %u\n", msg->header.message_revision);
    return -1;
  }

  // Look for device serial number in list of devices...
  if(serial_number != 0)  // when a specific serial is called out, ignore everything else
  {
    if(serial_number == msg->header.serial_num) //this should be our number
    {
      boost::mutex::scoped_lock(library_lock_);
      Devices[0]->message_time = ros::Time::now();
      Devices[0]->setTransitionMessage(*msg);
    }
  }
  else
  {
    for (unsigned i = 0; i<Devices.size(); ++i) {
      if (Devices[i]->getPowerMessage().header.serial_num == msg->header.serial_num) {
        boost::mutex::scoped_lock(library_lock_);
        Devices[i]->setTransitionMessage(*msg);
        return 0;
      }
    }

    // Add new device to list
    Device *newDevice = new Device();
    Devices.push_back(newDevice);
    newDevice->setTransitionMessage(*msg);
  }

  return 0;
}

// collect status packets for 0.5 seconds.  Keep the last packet sent
// from each seperate power device.
int PowerBoard::collect_messages()
{
  PowerMessage *power_msg;
  TransitionMessage *transition_msg;
  MessageHeader *header;
  char tmp_buf[256];  //bigger than our max size we expect

  //ROS_DEBUG("PowerMessage size=%u", sizeof(PowerMessage));
  //ROS_DEBUG("TransitionMessage size=%u", sizeof(TransitionMessage));

  timeval timeout;  //timeout once a second to check if we should die or not.
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  while (1)
  {
    // Wait for packets to arrive on socket.
    fd_set read_set;
    int max_sock = -1;
    FD_ZERO(&read_set);
    //for (unsigned i = 0; i<SendInterfaces.size(); ++i)
      ReceiveInterface->AddToReadSet(read_set,max_sock);

    int result = select(max_sock+1, &read_set, NULL, NULL, &timeout);

    //fprintf(stderr,"*");

    if (result == -1) {
      perror("Select");
      return -1;
    }
    else if (result == 0) {
      return 0;
    }
    else if (result >= 1) {
      Interface *recvInterface = ReceiveInterface;
#if 0
      for (unsigned i = 0; i<SendInterfaces.size(); ++i) {
        //figure out which interface we recieved on
        if (SendInterfaces[i]->IsReadSet(read_set)) {
          recvInterface = SendInterfaces[i];
          //ROS_INFO("Receive index=%d", i);
          break;
        }
      }
#endif

      //ROS_INFO("Receive on %s", inet_ntoa(((struct sockaddr_in *)(&recvInterface->interface.ifr_dstaddr))->sin_addr));
      int len = recv(recvInterface->recv_sock, tmp_buf, sizeof(tmp_buf), 0);
      if (len == -1) {
        ROS_ERROR("Error recieving on socket");
        return -1;
      }
      else if (len < (int)sizeof(MessageHeader)) {
        ROS_ERROR("recieved message of incorrect size %d\n", len);
      }

      header = (MessageHeader*)tmp_buf;
      {

        //ROS_DEBUG("Header type=%d", header->message_id);
        if(header->message_id == MESSAGE_ID_POWER)
        {
          power_msg = (PowerMessage*)tmp_buf;
          if (len == -1) {
            ROS_ERROR("Error recieving on socket");
            return -1;
          }
          else if (len != (int)sizeof(PowerMessage)) {
            ROS_ERROR("recieved message of incorrect size %d\n", len);
          }
          else {
            if (process_message(power_msg))
              return -1;
          }
        }
        else if(header->message_id == MESSAGE_ID_TRANSITION)
        {
          transition_msg = (TransitionMessage *)tmp_buf;
          if (len == -1) {
            ROS_ERROR("Error recieving on socket");
            return -1;
          }
          else if (len != sizeof(TransitionMessage)) {
            ROS_ERROR("recieved message of incorrect size %d\n", len);
          }
          else {
            if (process_transition_message(transition_msg))
              return -1;
          }
        }
        else
        {
          ROS_DEBUG("Discard message len=%d", len);
        }
      }
    }
    else {
      ROS_ERROR("Unexpected select result %d\n", result);
      return -1;
    }
  }

  return 0;
}

PowerBoard::PowerBoard( const ros::NodeHandle node_handle, unsigned int serial_number ) : node_handle(node_handle)
{
  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);

  if( my_logger->getLevel() == 0 )    //has anyone set our level??
  {
    // Set the ROS logger
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
  }

  this->serial_number = serial_number;
}

void PowerBoard::init()
{
  if(serial_number != 0)
  {
    ROS_INFO("PowerBoard: created with serial number = %d", serial_number);
    Device *newDevice = new Device();
    Devices.push_back(newDevice);
  }

  service = node_handle.advertiseService("power_board_control", &PowerBoard::commandCallback, this);

  pub = node_handle.advertise<diagnostic_msgs::DiagnosticMessage>("/diagnostics", 2);
}

bool PowerBoard::commandCallback(pr2_power_board::PowerBoardCommand::Request &req_,
                     pr2_power_board::PowerBoardCommand::Response &res_)
{
  res_.retval = send_command( req_.serial_number, req_.breaker_number, req_.command, req_.flags);

  return true;
}

void PowerBoard::collectMessages()
{
  while(node_handle.ok())
  {
    collect_messages();
    //ROS_DEBUG("*");
  }
}

void PowerBoard::sendDiagnostic()
{
  diagnostic_msgs::DiagnosticMessage msg_out;
  diagnostic_msgs::DiagnosticStatus stat;
  diagnostic_msgs::DiagnosticValue val;
  diagnostic_msgs::DiagnosticString strval;

  ros::Rate r(1);
  while(node_handle.ok())
  {
    r.sleep();
    //ROS_DEBUG("-");
    boost::mutex::scoped_lock(library_lock_);
  
    for (unsigned i = 0; i<Devices.size(); ++i)
    {
      msg_out.status.clear();
      stat.values.clear();
      stat.strings.clear();

      Device *device = Devices[i];
      const PowerMessage *pmesg = &device->getPowerMessage();
      
      ostringstream ss;
      ss << "Power board " << pmesg->header.serial_num;
      stat.name = ss.str();

      if( (ros::Time::now() - device->message_time) > TIMEOUT )
      {
        stat.level = 2;
        stat.message = "No Updates";
      }
      else
      {
        stat.level = 0;
        stat.message = "Running";
      }
      const StatusStruct *status = &pmesg->status;

      ROS_DEBUG("Device %u", i);
      ROS_DEBUG(" Serial       = %u", pmesg->header.serial_num);

      ss.str("");
      ss << pmesg->header.serial_num;
      strval.value = ss.str();
      strval.label = "Serial Number";
      stat.strings.push_back(strval);

      ROS_DEBUG(" Current      = %f", status->input_current);
      val.value = status->input_current;
      val.label = "Input Current";
      stat.values.push_back(val);

      ROS_DEBUG(" Voltages:");
      ROS_DEBUG("  Input       = %f", status->input_voltage);
      val.value = status->input_voltage;
      val.label = "Input Voltage";
      stat.values.push_back(val);
      ROS_DEBUG("  DCDC 12     = %f", status->DCDC_12V_out_voltage);
      val.value = status->DCDC_12V_out_voltage;
      val.label = "DCDC12";
      stat.values.push_back(val);
      ROS_DEBUG("  DCDC 15     = %f", status->DCDC_19V_out_voltage);
      val.value = status->DCDC_19V_out_voltage;
      val.label = "DCDC 15";
      stat.values.push_back(val);
      ROS_DEBUG("  CB0 (Base)  = %f", status->CB0_voltage);
      val.value = status->CB0_voltage;
      val.label = "Breaker 0 Voltage";
      stat.values.push_back(val);
      ROS_DEBUG("  CB1 (R-arm) = %f", status->CB1_voltage);
      val.value = status->CB1_voltage;
      val.label = "Breaker 1 Voltage";
      stat.values.push_back(val);
      ROS_DEBUG("  CB2 (L-arm) = %f", status->CB2_voltage);
      val.value = status->CB2_voltage;
      val.label = "Breaker 2 Voltage";
      stat.values.push_back(val);

      ROS_DEBUG(" Board Temp   = %f", status->ambient_temp);
      val.value = status->ambient_temp;
      val.label = "Board Temperature";
      stat.values.push_back(val);
      ROS_DEBUG(" Fan Speeds:");
      ROS_DEBUG("  Fan 0       = %u", status->fan0_speed);
      val.value = status->fan0_speed;
      val.label = "Fan 0 Speed";
      stat.values.push_back(val);
      ROS_DEBUG("  Fan 1       = %u", status->fan1_speed);
      val.value = status->fan1_speed;
      val.label = "Fan 1 Speed";
      stat.values.push_back(val);
      ROS_DEBUG("  Fan 2       = %u", status->fan2_speed);
      val.value = status->fan2_speed;
      val.label = "Fan 2 Speed";
      stat.values.push_back(val);
      ROS_DEBUG("  Fan 3       = %u", status->fan3_speed);
      val.value = status->fan3_speed;
      val.label = "Fan 3 Speed";
      stat.values.push_back(val);

      ROS_DEBUG(" State:");
      ROS_DEBUG("  CB0 (Base)  = %s", cb_state_to_str(status->CB0_state));
      strval.value = cb_state_to_str(status->CB0_state);
      strval.label = "Breaker 0 State";
      stat.strings.push_back(strval);
      ROS_DEBUG("  CB1 (R-arm) = %s", cb_state_to_str(status->CB1_state));
      strval.value = cb_state_to_str(status->CB1_state);
      strval.label = "Breaker 1 State";
      stat.strings.push_back(strval);
      ROS_DEBUG("  CB2 (L-arm) = %s", cb_state_to_str(status->CB2_state));
      strval.value = cb_state_to_str(status->CB2_state);
      strval.label = "Breaker 2 State";
      stat.strings.push_back(strval);
      ROS_DEBUG("  DCDC        = %s", master_state_to_str(status->DCDC_state));
      strval.value = master_state_to_str(status->DCDC_state);
      strval.label = "DCDC state";
      stat.strings.push_back(strval);

      ROS_DEBUG(" Status:");
      ROS_DEBUG("  CB0 (Base)  = %s", (status->CB0_status) ? "On" : "Off");
      strval.value = (status->CB0_status) ? "On" : "Off";
      strval.label = "Breaker 0 Status";
      stat.strings.push_back(strval);
      ROS_DEBUG("  CB1 (R-arm) = %s", (status->CB1_status) ? "On" : "Off");
      strval.value = (status->CB1_status) ? "On" : "Off";
      strval.label = "Breaker 1 Status";
      stat.strings.push_back(strval);
      ROS_DEBUG("  CB2 (L-arm) = %s", (status->CB2_status) ? "On" : "Off");
      strval.value = (status->CB2_status) ? "On" : "Off";
      strval.label = "Breaker 2 Status";
      stat.strings.push_back(strval);
      ROS_DEBUG("  estop_button= %x", (status->estop_button_status));
      val.value = status->estop_button_status;
      val.label = "RunStop Button Status";
      stat.values.push_back(val);
      ROS_DEBUG("  estop_status= %x", (status->estop_status));
      val.value = status->estop_status;
      val.label = "RunStop Status";
      stat.values.push_back(val);

      ROS_DEBUG(" Revisions:");
      ROS_DEBUG("         PCA = %c", status->pca_rev);
      ss.str("");
      ss << status->pca_rev;
      strval.value = ss.str();
      strval.label = "PCA";
      stat.strings.push_back(strval);
      ROS_DEBUG("         PCB = %c", status->pcb_rev);
      ss.str("");
      ss << status->pcb_rev;
      strval.value = ss.str();
      strval.label = "PCB";
      stat.strings.push_back(strval);
      ROS_DEBUG("       Major = %c", status->major_rev);
      ss.str("");
      ss << status->major_rev;
      strval.value = ss.str();
      strval.label = "Major Revision";
      stat.strings.push_back(strval);
      ROS_DEBUG("       Minor = %c", status->minor_rev);
      ss.str("");
      ss << status->minor_rev;
      strval.value = ss.str();
      strval.label = "Minor Revision";
      stat.strings.push_back(strval);

      val.value = status->min_input_voltage;
      val.label = "Min Voltage";
      stat.values.push_back(val);
      val.value = status->max_input_current;
      val.label = "Max Current";
      stat.values.push_back(val);


      const TransitionMessage *tmsg = &device->getTransitionMessage();
      for(int cb_index=0; cb_index < 3; ++cb_index)
      {
        const TransitionCount *trans = &tmsg->cb[cb_index];
        ROS_DEBUG("Transition: CB%d", cb_index);
        val.value = trans->stop_count;
        ss.str("");
        ss << "CB" << cb_index << " Stop Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->estop_count;
        ss.str("");
        ss << "CB" << cb_index << " E-Stop Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->trip_count;
        ss.str("");
        ss << "CB" << cb_index << " Trip Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->fail_18V_count;
        ss.str("");
        ss << "CB" << cb_index << " 18V Fail Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->disable_count;
        ss.str("");
        ss << "CB" << cb_index << " Disable Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->start_count;
        ss.str("");
        ss << "CB" << cb_index << " Start Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->pump_fail_count;
        ss.str("");
        ss << "CB" << cb_index << " Pump Fail Count";
        val.label = ss.str();
        stat.values.push_back(val);
        val.value = trans->reset_count;
        ss.str("");
        ss << "CB" << cb_index << " Reset Count";
        val.label = ss.str();
        stat.values.push_back(val);
      }

      msg_out.status.push_back(stat);

      //ROS_DEBUG("Publishing ");
      pub.publish(msg_out);
    }

  }
}

void getMessages()
{
  myBoard->collectMessages();
}

void sendMessages()
{
  myBoard->sendDiagnostic();
}

// CloseAll
void CloseAllInterfaces(void) {
  for (unsigned i=0; i<SendInterfaces.size(); ++i){
    if (SendInterfaces[i] != NULL) {
      delete SendInterfaces[i];
    }
  }

  delete ReceiveInterface;
}
void CloseAllDevices(void) {
  for (unsigned i=0; i<Devices.size(); ++i){
    if (Devices[i] != NULL) {
      delete Devices[i];
    }
  }
}

void setupReceive()
{
  Interface *newInterface = new Interface("rcv");
  newInterface->InitReceive();
  ReceiveInterface = newInterface;
}


// Build list of interfaces
int CreateAllInterfaces(void)
{
  //
  int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == -1) {
    ROS_ERROR("Couldn't create socket for ioctl requests");
    return -1;
  }

  struct ifconf get_io;
  get_io.ifc_req = new ifreq[10];
  get_io.ifc_len = sizeof(ifreq) * 10;

  if(ioctl( sock, SIOCGIFCONF, &get_io ) == 0)
  {
    int num_interfaces = get_io.ifc_len / sizeof(ifreq);
    ROS_DEBUG("Got %d interfaces", num_interfaces);
    for(int yy=0; yy < num_interfaces; ++yy)
    {
      ROS_DEBUG("interface=%s", get_io.ifc_req[yy].ifr_name);
      if(get_io.ifc_req[yy].ifr_addr.sa_family == AF_INET)
      {
        //ROS_DEBUG("ioctl: family=%d", get_io.ifc_req[yy].ifr_addr.sa_family);
        sockaddr_in *addr = (sockaddr_in*)&get_io.ifc_req[yy].ifr_addr;
        ROS_DEBUG("address=%s", inet_ntoa(addr->sin_addr) );

        if ((strncmp("lo", get_io.ifc_req[yy].ifr_name, strlen(get_io.ifc_req[yy].ifr_name)) == 0) ||
            (strncmp("tun", get_io.ifc_req[yy].ifr_name, 3) == 0) ||
            (strncmp("vmnet", get_io.ifc_req[yy].ifr_name, 5) == 0))
        {
          ROS_INFO("Ignoring interface %*s",strlen(get_io.ifc_req[yy].ifr_name), get_io.ifc_req[yy].ifr_name);
          continue;
        }
        else
        {
          ROS_INFO("Found interface    %*s",strlen(get_io.ifc_req[yy].ifr_name), get_io.ifc_req[yy].ifr_name);
          Interface *newInterface = new Interface(get_io.ifc_req[yy].ifr_name);
          assert(newInterface != NULL);
          if (newInterface == NULL)
          {
            continue;
          }


          struct ifreq *ifr = &get_io.ifc_req[yy];
          if(ioctl(sock, SIOCGIFBRDADDR, ifr) == 0)
          {
            if (ifr->ifr_broadaddr.sa_family == AF_INET)
            {
              struct sockaddr_in *br_addr = (struct sockaddr_in *) &ifr->ifr_dstaddr;

              ROS_DEBUG ("Broadcast addess %s", inet_ntoa(br_addr->sin_addr));

              if (newInterface->Init(addr, br_addr))
              {
                ROS_ERROR("Error initializing interface %*s", sizeof(get_io.ifc_req[yy].ifr_name), get_io.ifc_req[yy].ifr_name);
                delete newInterface;
                newInterface = NULL;
                continue;
              }
              else
              {
                // Interface is good add it to interface list
                SendInterfaces.push_back(newInterface);

              }

            }

          }


        }
      }
    }
  }
  else
  {
    ROS_ERROR("Bad ioctl status");
  }

  delete[] get_io.ifc_req;

  setupReceive();
  //ROS_INFO("Found %d usable interfaces", Interfaces.size());

  return 0;
}

int main(int argc, char** argv)
{
  unsigned int serial_option;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "this help message")
    ("serial", po::value<unsigned int>(&serial_option)->default_value(0), "filter a specific serial number");

  po::variables_map vm;
  po::store(po::parse_command_line( argc, argv, desc), vm);
  po::notify(vm);

  if( vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  ros::init(argc, argv, "PowerBoard");

  CreateAllInterfaces();

  ros::NodeHandle handle;
  myBoard = new PowerBoard( handle, serial_option);
  myBoard->init();

  boost::thread getThread( &getMessages );
  boost::thread sendThread( &sendMessages );

  ros::spin(); //wait for ros to shut us down

  sendThread.join();
  getThread.join();

  CloseAllDevices();
  CloseAllInterfaces();

  
  delete myBoard;
  return 0;

}
