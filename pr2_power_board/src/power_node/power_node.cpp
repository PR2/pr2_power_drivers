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
#include "diagnostic_msgs/DiagnosticArray.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "pr2_msgs/PowerBoardState.h"
#include "rosconsole/macros_generated.h"
#include "ros/ros.h"

using namespace std;
namespace po = boost::program_options;

// Keep a pointer to the last message received for
// Each board.
static Device *devicePtr;
static Interface *SendInterface;
static Interface *ReceiveInterface;
static PowerBoard *myBoard;

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

  if(newpmsg.header.message_revision == 2) //migrate old messages to new structure
  {
    memcpy( &pmsg.header, &newpmsg.header, sizeof(MessageHeader));

    //Copy the contents of the Rev2 message into the current structure.
    memcpy( &pmsg.status, &newpmsg.status, sizeof(StatusStruct_Rev2));
  }
  else
    pmsg = newpmsg;

}

Interface::Interface() : recv_sock(-1), send_sock(-1) 
{

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


int Interface::InitReceive(const std::string &address_str)
{

  recv_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (recv_sock == -1) {
    perror("Couldn't create recv socket");
    return -1;
  }

 // Allow reuse of receive port
  int opt = 1;
  if (setsockopt(recv_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("Couldn't set reuse addr on recv socket\n");
    Close();
    return -1;
  }

#if 0
  // Allow broadcast on send socket
  opt = 1;
  if (setsockopt(recv_sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt))) {
    perror("Setting broadcast option on recv");
    Close();
    return -1;
  }
#endif

  // Bind socket to receive packets on <UDP_STATUS_PORT> from any address/interface
  struct sockaddr_in sin;
  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_port = htons(POWER_PORT);
  sin.sin_addr.s_addr = (INADDR_ANY);
  //inet_pton( AF_INET, address_str.c_str(), &sin.sin_addr);
  if (bind(recv_sock, (struct sockaddr*)&sin, sizeof(sin))) {
    perror("Couldn't Bind socket to port");
    Close();
    return -1;
  }

  return 0;
}

int Interface::Init( const std::string &address_str)
{

  send_sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (send_sock == -1) {
    Close();
    perror("Couldn't create send socket");
    return -1;
  }


  int opt;
 
 // Allow reuse of receive port
  opt = 1;
  if (setsockopt(send_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("Error allowing socket reuse");
    Close();
    return -1;
  }

  struct sockaddr_in sin;
  memset(&sin, 0, sizeof(sin));
  sin.sin_family = AF_INET;

  sin.sin_port = htons(POWER_PORT);
  //sin.sin_addr.s_addr = inet_addr("192.168.13.19");
  inet_pton( AF_INET, address_str.c_str(), &sin.sin_addr);
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
  assert(devicePtr != NULL);

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
  cmdmsg.header.message_revision = COMMAND_MESSAGE_REVISION;
  cmdmsg.header.message_id = MESSAGE_ID_COMMAND;
  cmdmsg.header.serial_num = devicePtr->getPowerMessage().header.serial_num;
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
  //ROS_INFO("Send on %s", inet_ntoa(SendInterfaces[xx]->ifc_address.sin_addr));
  int result = send(SendInterface->send_sock, &cmdmsg, sizeof(cmdmsg), 0);
  if (result == -1) {
    ROS_ERROR("Error sending");
    return -1;
  } else if (result != sizeof(cmdmsg)) {
    ROS_WARN("Error sending : send only took %d of %d bytes\n",
            result, sizeof(cmdmsg));
  }

  ROS_DEBUG("Send to Serial=%u, revision=%u", cmdmsg.header.serial_num, cmdmsg.header.message_revision);
  ROS_DEBUG("Sent command %s(%d), circuit %d", command.c_str(), command_enum, circuit_breaker);

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
int PowerBoard::process_message(const PowerMessage *msg, int len)
{
  if ((msg->header.message_revision > CURRENT_MESSAGE_REVISION) || (msg->header.message_revision < MINIMUM_MESSAGE_REVISION)) {
    ROS_WARN("Got message with incorrect message revision %u\n", msg->header.message_revision);
    return -1;
  }

  if ((msg->header.message_revision == CURRENT_MESSAGE_REVISION) && (len != CURRENT_MESSAGE_SIZE))
    ROS_ERROR("received message of incorrect size %d for rev=%d\n", len, msg->header.message_revision);

  if ((msg->header.message_revision == MINIMUM_MESSAGE_REVISION) && (len != REVISION_2_MESSAGE_SIZE))
    ROS_ERROR("received message of incorrect size %d for rev=%d\n", len, msg->header.message_revision);


  {
    boost::mutex::scoped_lock(library_lock_);
    devicePtr->message_time = ros::Time::now();
    devicePtr->setPowerMessage(*msg);
  }

  return 0;
}

int PowerBoard::process_transition_message(const TransitionMessage *msg, int len)
{
  if (msg->header.message_revision != TRANSITION_MESSAGE_REVISION) {
    ROS_WARN("Got message with incorrect message revision %u\n", msg->header.message_revision);
    return -1;
  }

  if (len != sizeof(TransitionMessage)) {
    ROS_ERROR("received message of incorrect size %d\n", len);
    return -2;
  }

  {
    boost::mutex::scoped_lock(library_lock_);
    devicePtr->message_time = ros::Time::now();
    devicePtr->setTransitionMessage(*msg);
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

      //ROS_INFO("Receive on %s", inet_ntoa(((struct sockaddr_in *)(&recvInterface->interface.ifr_dstaddr))->sin_addr));
      int len = recv(recvInterface->recv_sock, tmp_buf, sizeof(tmp_buf), 0);
      if (len == -1) {
        ROS_ERROR("Error recieving on socket");
        return -1;
      }
      else if (len < (int)sizeof(MessageHeader)) {
        ROS_ERROR("received message of incorrect size %d\n", len);
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
/*
          else if (len != (int)sizeof(PowerMessage)) {
            ROS_ERROR("received message of incorrect size %d\n", len);
          }
*/
          else {
            if (process_message(power_msg, len))
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
          else {
            if (process_transition_message(transition_msg, len))
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

PowerBoard::PowerBoard( const ros::NodeHandle node_handle ) : node_handle(node_handle)
{
  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  fprintf(stderr, "Logger Name: %s\n", ROSCONSOLE_DEFAULT_NAME);

  if( my_logger->getLevel() == 0 )    //has anyone set our level??
  {
    // Set the ROS logger
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
  }

}

void PowerBoard::init()
{
  //ROS_INFO("PowerBoard: created with serial number = %d", serial_number);
  devicePtr = new Device();

  service = node_handle.advertiseService("power_board_control", &PowerBoard::commandCallback, this);

  diags_pub = node_handle.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 2);
  state_pub = node_handle.advertise<pr2_msgs::PowerBoardState>("/power_board_state", 2);
}

bool PowerBoard::commandCallback(pr2_power_board::PowerBoardCommand::Request &req_,
                     pr2_power_board::PowerBoardCommand::Response &res_)
{
  res_.retval = send_command( req_.serial_number, req_.breaker_number, req_.command, req_.flags);
  requestMessage();

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

void PowerBoard::sendMessages()
{
  ros::Rate r(1);
  while(node_handle.ok())
  {
    r.sleep();
    //ROS_DEBUG("-");
    boost::mutex::scoped_lock(library_lock_);
  
    {
      diagnostic_msgs::DiagnosticArray msg_out;
      diagnostic_updater::DiagnosticStatusWrapper stat;

      const PowerMessage *pmesg = &devicePtr->getPowerMessage();
      
      ostringstream ss;
      ss << "Power board " << pmesg->header.serial_num;
      stat.name = ss.str();

      if( (ros::Time::now() - devicePtr->message_time) > TIMEOUT )
      {
        stat.summary(2, "No Updates");
      }
      else
      {
        stat.summary(0, "Running");
      }
      const StatusStruct *status = &pmesg->status;

      //ROS_DEBUG("Device %u", i);
      //ROS_DEBUG(" Serial       = %u", pmesg->header.serial_num);

      stat.add("Serial Number", pmesg->header.serial_num);

      //ROS_DEBUG(" Current      = %f", status->input_current);
      stat.add("Input Current", status->input_current);

      //ROS_DEBUG(" Voltages:");
      //ROS_DEBUG("  Input       = %f", status->input_voltage);
      stat.add("Input Voltage", status->input_voltage);
      
      //ROS_DEBUG("  DCDC 12 aux = %f", status->DCDC_12V_aux);
      stat.add("DCDC 12V aux", status->DCDC_12V_aux);
      
      //ROS_DEBUG("  DCDC 12V cpu0   = %f", status->DCDC_12V_cpu0);
      stat.add("DCDC 12V cpu0", status->DCDC_12V_cpu0);
      
      //ROS_DEBUG("  CB0 (Base)  = %f", status->CB0_voltage);
      stat.add("Breaker 0 Voltage", status->CB0_voltage);
      
      //ROS_DEBUG("  CB1 (R-arm) = %f", status->CB1_voltage);
      stat.add("Breaker 1 Voltage", status->CB1_voltage);
      
      //ROS_DEBUG("  CB2 (L-arm) = %f", status->CB2_voltage);
      stat.add("Breaker 2 Voltage", status->CB2_voltage);

      //ROS_DEBUG(" Board Temp   = %f", status->ambient_temp);
      stat.add("Board Temperature", status->ambient_temp);
      
      //ROS_DEBUG(" Fan Speeds:");
      //ROS_DEBUG("  Fan 0       = %u", status->fan0_speed);
      stat.add("Fan 0 Speed", status->fan0_speed);
      
      //ROS_DEBUG("  Fan 1       = %u", status->fan1_speed);
      stat.add("Fan 1 Speed", status->fan1_speed);
      
      //ROS_DEBUG("  Fan 2       = %u", status->fan2_speed);
      stat.add("Fan 2 Speed", status->fan2_speed);
      
      //ROS_DEBUG("  Fan 3       = %u", status->fan3_speed);
      stat.add("Fan 3 Speed", status->fan3_speed);

      //ROS_DEBUG(" State:");
      //ROS_DEBUG("  CB0 (Base)  = %s", cb_state_to_str(status->CB0_state));
      stat.add("Breaker 0 State", cb_state_to_str(status->CB0_state));

      //ROS_DEBUG("  CB1 (R-arm) = %s", cb_state_to_str(status->CB1_state));
      stat.add("Breaker 1 State", cb_state_to_str(status->CB1_state));

      //ROS_DEBUG("  CB2 (L-arm) = %s", cb_state_to_str(status->CB2_state));
      stat.add("Breaker 2 State", cb_state_to_str(status->CB2_state));
      
      //ROS_DEBUG("  DCDC        = %s", master_state_to_str(status->DCDC_state));
      stat.add("DCDC state", master_state_to_str(status->DCDC_state));

      //ROS_DEBUG(" Status:");
      //ROS_DEBUG("  CB0 (Base)  = %s", (status->CB0_status) ? "On" : "Off");
      stat.add("Breaker 0 Status", (status->CB0_status) ? "On" : "Off");
      
      //ROS_DEBUG("  CB1 (R-arm) = %s", (status->CB1_status) ? "On" : "Off");
      stat.add("Breaker 1 Status", (status->CB1_status) ? "On" : "Off");
      
      //ROS_DEBUG("  CB2 (L-arm) = %s", (status->CB2_status) ? "On" : "Off");
      stat.add("Breaker 2 Status", (status->CB2_status) ? "On" : "Off");
      
      //ROS_DEBUG("  estop_button= %x", (status->estop_button_status));
      stat.add("RunStop Button Status", (status->estop_button_status ? "True":"False"));
      
      //ROS_DEBUG("  estop_status= %x", (status->estop_status));
      stat.add("RunStop Status", (status->estop_status ? "True":"False"));

      //ROS_DEBUG(" Revisions:");
      //ROS_DEBUG("         PCA = %c", status->pca_rev);
      stat.add("PCA", status->pca_rev);

      //ROS_DEBUG("         PCB = %c", status->pcb_rev);
      stat.add("PCB", status->pcb_rev);

      //ROS_DEBUG("       Major = %c", status->major_rev);
      stat.add("Major Revision", status->major_rev);
      
      //ROS_DEBUG("       Minor = %c", status->minor_rev);
      stat.add("Minor Revision", status->minor_rev);

      stat.add("Min Voltage", status->min_input_voltage);
      stat.add("Max Current", status->max_input_current);

      //ROS_DEBUG("  DCDC 12V cpu1   = %f", status->DCDC_12V_cpu1);
      stat.add("DCDC 12V cpu1", status->DCDC_12V_cpu1);

      //ROS_DEBUG("  DCDC 12V user   = %f", status->DCDC_12V_user);
      stat.add("DCDC 12V user", status->DCDC_12V_user);
      
      for( int xx = 0; xx < 4; ++xx)
      {
        //ROS_DEBUG("  Battery %d voltage=%f", xx, status->battery_voltage[xx]);
        ss.str("");
        ss << "Battery " << xx << " voltage=";
        stat.add(ss.str(), status->battery_voltage[xx]);
      }
      
      
      const TransitionMessage *tmsg = &devicePtr->getTransitionMessage();
      for(int cb_index=0; cb_index < 3; ++cb_index)
      {
        const TransitionCount *trans = &tmsg->cb[cb_index];
        //ROS_DEBUG("Transition: CB%d", cb_index);
        ss.str("");
        ss << "CB" << cb_index << " Stop Count";
        stat.add(ss.str(), (int)trans->stop_count);
        ////ROS_DEBUG("  Stop Count=%d", trans->stop_count);
        
        ss.str("");
        ss << "CB" << cb_index << " E-Stop Count";
        stat.add(ss.str(), (int)trans->estop_count);
        
        ss.str("");
        ss << "CB" << cb_index << " Trip Count";
        stat.add(ss.str(), (int)trans->trip_count);
        
        ss.str("");
        ss << "CB" << cb_index << " 18V Fail Count";
        stat.add(ss.str(), (int)trans->fail_18V_count);
      
        ss.str("");
        ss << "CB" << cb_index << " Disable Count";
        stat.add(ss.str(), (int)trans->disable_count);
      
        ss.str("");
        ss << "CB" << cb_index << " Start Count";
        stat.add(ss.str(), (int)trans->start_count);
      
        ss.str("");
        ss << "CB" << cb_index << " Pump Fail Count";
        stat.add(ss.str(), (int)trans->pump_fail_count);
      
        ss.str("");
        ss << "CB" << cb_index << " Reset Count";
        stat.add(ss.str(), (int)trans->reset_count);
      }

      msg_out.status.push_back(stat);

      //ROS_DEBUG("Publishing ");
      diags_pub.publish(msg_out);

      pr2_msgs::PowerBoardState state_msg;
      state_msg.name = stat.name;
      state_msg.serial_num = pmesg->header.serial_num;
      state_msg.input_voltage = status->input_voltage;
      state_msg.circuit_voltage[0] = status->CB0_voltage;
      state_msg.circuit_voltage[1] = status->CB1_voltage;
      state_msg.circuit_voltage[2] = status->CB2_voltage;
      state_msg.circuit_state[0] = status->CB0_state;
      state_msg.circuit_state[1] = status->CB1_state;
      state_msg.circuit_state[2] = status->CB2_state;
      state_msg.run_stop = status->estop_status;
      state_msg.wireless_stop = status->estop_button_status;
      state_pub.publish(state_msg);
    }
  }
}

int PowerBoard::requestMessage()
{

  GetMessage cmdmsg;
  memset(&cmdmsg, 0, sizeof(cmdmsg));
  cmdmsg.header.message_revision = STATUS_MESSAGE_REVISION;
  cmdmsg.header.message_id = MESSAGE_ID_STATUS;
  cmdmsg.header.serial_num = devicePtr->getPowerMessage().header.serial_num;
  strncpy(cmdmsg.header.text, "power status message", sizeof(cmdmsg.header.text));

  cmdmsg.message_to_get = MESSAGE_ID_POWER;

  errno = 0;
  int result = send(SendInterface->send_sock, &cmdmsg, sizeof(cmdmsg), 0);
  if (result == -1) {
    ROS_ERROR("Error sending");
    return -1;
  } else if (result != sizeof(cmdmsg)) {
    ROS_WARN("Error sending : send only took %d of %d bytes\n", result, sizeof(cmdmsg));
  }

  ROS_DEBUG("requestMessage: to Serial=%u, revision=%u", cmdmsg.header.serial_num, cmdmsg.header.message_revision);
  return 0;
}


void getMessages()
{
  myBoard->collectMessages();
}

void sendMessages()
{
  myBoard->sendMessages();
}

// CloseAll
void CloseAllInterfaces(void) 
{

  delete SendInterface;
  delete ReceiveInterface;
}

void CloseAllDevices(void) 
{
  delete devicePtr;
}


// Build list of interfaces
int CreateAllInterfaces(const std::string &address_str)
{
  //
  int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock == -1) {
    ROS_ERROR("Couldn't create socket for ioctl requests");
    return -1;
  }

  SendInterface = new Interface();
  if (SendInterface->Init(address_str))
  {
    ROS_ERROR("Error initializing interface");
    delete SendInterface;
    SendInterface = NULL;
  }


  ReceiveInterface  = new Interface();
  ReceiveInterface->InitReceive(address_str);

  return 0;
}

int main(int argc, char** argv)
{
  std::string address_str;
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "this help message")
    ("address", po::value<std::string>(&address_str)->default_value("0.0.0.0"), "IP address for specific Power Board");

  po::variables_map vm;
  po::store(po::parse_command_line( argc, argv, desc), vm);
  po::notify(vm);

  if( vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }

  ROS_INFO("PowerNode:Using Address=%s", address_str.c_str());
  if( address_str.compare("0.0.0.0") == 0 )
  {
    ROS_ERROR("PowerNode: Error you did not specify the IP address");
    exit(-1);
  }

  ros::init(argc, argv, "PowerBoard");

  CreateAllInterfaces(address_str);

  ros::NodeHandle handle;
  myBoard = new PowerBoard( handle );
  myBoard->init();

  boost::thread getThread( &getMessages );
  boost::thread sendThread( &sendMessages );

  double sample_frequency = 10; //In Hertz
  handle.getParam( "/power_node/sample_frequency", sample_frequency );
  ROS_INFO("Using sampling frequency %fHz", sample_frequency);

  ros::Time last_time = ros::Time::now();

  double ros_rate = 10;

  if(sample_frequency > ros_rate)
    ros_rate = sample_frequency;

  ros::Rate r(ros_rate);
  const ros::Duration MSG_RATE(1/sample_frequency);

  while(handle.ok())
  {
    r.sleep();
    if( (ros::Time::now() - last_time) > MSG_RATE )
    {
      myBoard->requestMessage();
      //ROS_INFO("Send ");
      last_time = ros::Time::now();
    }
    ros::spinOnce();
  }

  sendThread.join();
  getThread.join();

  CloseAllDevices();
  CloseAllInterfaces();

  
  delete myBoard;
  return 0;

}

Device::Device(): message_time(0,0)
{ 
  pmsgset = false; 
  tmsgset = false; 
  memset( &pmsg, 0, sizeof(PowerMessage)); 
  memset( &tmsg, 0, sizeof(TransitionMessage)); 
};
