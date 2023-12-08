#ifndef _PPS_NODE_HPP_
#define _PPS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <base_interface/msg/ops.hpp>
#include <cstdio>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <signal.h>
#include <string>
#include <string.h>
#include <iostream>
#include <poll.h>
#include <fstream>
#include <sys/time.h>

#define RX_PPS_DATA_LEN 24

class ppsCommSerial
{
private:
  /* data */
  struct termios termAttr;
  struct sigaction saio;
  int ifSerial = 0;
  const char *this_port;
  uint32_t this_baudrate;
  int handle;
public:
  int pps_Init(int &fd, char *port, int baudrate);
};

class PPSNode :public rclcpp::Node
{
public:
  PPSNode();
  void PPSCallback();
  
  rclcpp::Publisher<base_interface::msg::Ops>::SharedPtr ops_pub_;
};

#endif