#include <pps/pps_node.hpp>

int PPS_handle;

ppsCommSerial pps;

union trsdata
{  
  char dataC[RX_PPS_DATA_LEN];
  float dataF[RX_PPS_DATA_LEN / 4];
}ppsData;

PPSNode::PPSNode() : Node("pps_node")
{
  ops_pub_ = this->create_publisher<base_interface::msg::Ops>("/ops_data", 10);
}

void PPSNode::PPSCallback()
{
  auto msg = base_interface::msg::Ops();
  char ch;
  int pps_len;
  int count = 0;
  int dataCnt = 0;
  while (PPS_handle) 
  {
    pps_len = read(PPS_handle, &ch, 1);
    if (pps_len > 0) 
    {
      switch (count)
      {
        case 0:           //判断帧头0x0d
        {
          if (ch == 0x0d) 
          {
            count++;
          }
          else 
          {
            count = 0;
          }
          break;
        }
        case 1:           //判断帧头0x0a
        {
          if (ch == 0x0a) 
          {
            dataCnt = 0;
            count++;
          }
          else if (ch != 0x0d)
          {
            count = 0;
          }
          break;
        }
        case 2:           //接收定位系统发数
        {
          ppsData.dataC[dataCnt] = ch;
          dataCnt++;
          if (dataCnt >= RX_PPS_DATA_LEN)
          {
            dataCnt = 0;
            count++;
          }
          break;
        }
        case 3:           //判断帧尾0x0a
        {
          if (ch == 0x0a) 
          {
            count++;
          }
          else
          {
            count = 0;
          }
          break;
        }
        case 4:           //判断帧尾0x0d
        {
          if (ch == 0x0d) 
          {
            msg.theta = ppsData.dataF[0];
            msg.x = -(ppsData.dataF[3] + 115 * sin(msg.theta/180*3.14159265));
            msg.y = -(ppsData.dataF[4] - 115 * cos(msg.theta/180*3.14159265) + 115);
            RCLCPP_INFO(this->get_logger(),"theta=%f,msg.x = %f,msg.y = %f",msg.theta,msg.x,msg.y);
            ops_pub_->publish(msg);//发布数据
          }
          count = 0;
          break;
        }
        default:
        {
          count = 0;
          break;
        }
      }
    }        
  }     
}

//初始化定位系统串口
int ppsCommSerial::pps_Init(int& fd, char* port, int baudrate)
{
  std::cout << "init " << port << " ...\n";
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  std::cout << fd << std::endl;
  if (fd == -1)
  {
    perror("open_port: Unable to open\n");
    return 0;
  }
  tcgetattr(fd, &termAttr);
  bzero(&termAttr, sizeof(termAttr));
  cfsetispeed(&termAttr, baudrate);
  cfsetospeed(&termAttr, baudrate);
  termAttr.c_cflag &= ~PARENB;
  termAttr.c_cflag &= ~CSTOPB;
  termAttr.c_cflag &= ~CSIZE;
  termAttr.c_cflag |= CS8;
  termAttr.c_cflag |= (CLOCAL | CREAD);
  termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termAttr.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | ISTRIP);
  termAttr.c_oflag &= ~OPOST;
  termAttr.c_cc[VMIN] = 1;
  termAttr.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &termAttr);
  tcflush(fd, TCIOFLUSH);
  std::cout << port << ": serial init ok!" << std::endl;
  return fd;
}

void sigint_handler(int signum) {
  std::cout << "Received SIGINT signal (" << signum << ")" << std::endl;
  exit(0);
}

int main(int argc, char ** argv)
{
  //接收ctrl+c信号
  signal(SIGINT, sigint_handler);

  //初始化ros2
  rclcpp::init(argc, argv);
  std::cout << "pps embedded init .." << std::endl;
  PPS_handle = pps.pps_Init(PPS_handle, "/dev/pps_uart", B115200);
  std::cout << "port init done!!!" << std::endl;

  //持续发布定位系统数据
  auto node = std::make_shared<PPSNode>();
  while (rclcpp::ok())
  {
    node->PPSCallback();
  }

  //释放资源
  rclcpp::shutdown();

  return 0;
}