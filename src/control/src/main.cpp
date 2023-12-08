#include <control/pathfollowing.hpp>
#include <control/debugfile.hpp>
#include <rclcpp/rclcpp.hpp>

commSerial *commserial = new commSerial("/dev/com_port", B115200);//串口通信类


//从MCU中读取数据并判断是否连接
void ReadMCU()
{
  MCUHB++;
  commserial->CommunicateRMcu();
  if(MCUHB > 20)
  {
    std::cout << "Connect Error" <<std::endl;
  }
}

//将目标轮子速度发至MCU
void SendMCU(float v1,float v2,float v3)
{
  commserial->tx_data.wheel_1_vel = v1;
  commserial->tx_data.wheel_2_vel = v2;
  commserial->tx_data.wheel_3_vel = v3;
  commserial->CommunicateSMcu();
}

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<controller>();                     //定义节点
  node->SysInit();

  while(rclcpp::ok())
  {
    if(timeFlag_5ms)
    {
      timeFlag_5ms = 0;

#ifndef DEBUG
      ReadMCU();
#endif

      node->actWheel_1 = commserial->rx_data.MCUWheel_1;
      node->actWheel_2 = commserial->rx_data.MCUWheel_2;
      node->actWheel_3 = commserial->rx_data.MCUWheel_3;

#ifndef DEBUG
      rclcpp::spin_some(node);
#else
      std::cout << "CallBack" << std::endl;
      std::shared_ptr<base_interface::msg::Ops> RxMsg = std::make_shared<base_interface::msg::Ops>();
      RxMsg->x=0;
      RxMsg->y=0;
      RxMsg->theta=0;
      node->RxCallBack(RxMsg);
#endif

#ifndef DEBUG
      SendMCU(node->wheel_1,node->wheel_2,node->wheel_3);
#endif

      Debug(node);

      node->lastpos = node->pos;
    }
  }
  CloseFile();
  rclcpp::shutdown();
  return 0;
}