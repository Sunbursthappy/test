#include <control/com.hpp>

//MCU心脏包
int MCUHB;

pthread_mutex_t mtx;

commSerial::commSerial(const char *port, uint32_t baudrat)
{
  this_port = port;
  this_baudrate = baudrat;
}

commSerial::~commSerial()
{

}
//初始化
void commSerial::commSerialInit()
{
  handle = open(this_port, O_RDWR | O_NOCTTY | O_NDELAY);
  while (handle == -1)
  {
    handle = open(this_port, O_RDWR | O_NOCTTY | O_NDELAY);
    perror("open_port: Unable to open serial\n");
  }
  tcgetattr(handle, &termAttr);
  cfsetispeed(&termAttr, this_baudrate);
  cfsetospeed(&termAttr, this_baudrate);
  termAttr.c_cflag &= ~PARENB;
  termAttr.c_cflag &= ~CSTOPB;
  termAttr.c_cflag &= ~CSIZE;
  termAttr.c_cflag |= CS8;
  termAttr.c_cflag |= (CLOCAL | CREAD);
  termAttr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termAttr.c_iflag &= ~(IXON | IXOFF | IXANY);
  termAttr.c_oflag &= ~OPOST;
  termAttr.c_cc[VMIN] = 0;
  termAttr.c_cc[VTIME] = 1;
  tcflush(handle, TCIOFLUSH);
  tcsetattr(handle, TCSANOW, &termAttr);
}
//检查串口连接
int commSerial::commSerialIsopen()
{
  return (ifSerial);
}
//接收数据到rxbuf并返回长度
int commSerial::commSerialRead() 
{
  int len = 0;
  len = read(handle, rxbuf, RX_DATA_LEN); 
  return len;
}
//发送txbuf中数据到MCU
void commSerial::commSerialWrite()
{
  write(handle, txbuf, TX_DATA_LEN);
}
//等待读取完成
void commSerial::WaitForMcuPrepared(void)
{
  while(commSerialRead() != RX_DATA_LEN){}
}
//发送函数
//包含6位帧头帧尾，3个float
//共18位
void commSerial::CommunicateSMcu()
{
  //帧头
  txbuf[0] = 'H';
  txbuf[1] = 'M';
  //发送数据：三个轮子期望速度
  trans_t trans;
  trans.dataf[0] = tx_data.wheel_1_vel;
  trans.dataf[1] = tx_data.wheel_2_vel;
  trans.dataf[2] = tx_data.wheel_3_vel;
  for (int j = 0; j < TX_DATA_FLOAT_NUM*4; j++),                        
  {
    txbuf[j+2] = trans.datac[j];
  }
  //帧尾
  txbuf[TX_DATA_LEN - 2] = '\r';
  txbuf[TX_DATA_LEN - 1] = '\n';
  //发数
  commSerialWrite();
}
//接收函数
//包含6位帧头帧尾，1个float
//共10位
void commSerial::CommunicateRMcu()
{
  //读数
  commSerialRead();
  //判断帧头帧尾
  if(rxbuf[0] == 'S' && rxbuf[1] == 'M'
  && rxbuf[RX_DATA_LEN-2] == 'm' 
  && rxbuf[RX_DATA_LEN-1] == 's' )
  {
    //读取数据，是否接收到MCU发来信息
    trans_r trans;
    for (int j = 0; j < RX_DATA_FLOAT_NUM*4 ; j++)
    {
      trans.datac[j] = rxbuf[j+2];
    }
    rx_data.MCUWheel_1 = trans.dataf[0];
    rx_data.MCUWheel_2 = trans.dataf[1];
    rx_data.MCUWheel_3 = trans.dataf[2];
    MCUHB = 0;
  }
  //清空暂存数组
  memset(rxbuf,0,sizeof(rxbuf));
}