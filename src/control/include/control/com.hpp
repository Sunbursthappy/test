#ifndef _COM_HPP_
#define _COM_HPP_

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

/**
 * 小电脑向mcu发送数据，包含3个float和4位帧头帧尾
 * 共16位
*/
#define TX_DATA_INT_NUM 0
#define TX_DATA_FLOAT_NUM 3
#define TX_DATA_LEN 16

/**
 * 小电脑接收mcu数据，包含3个float和4位帧头帧尾
 * 共16位
*/
#define RX_DATA_INT_NUM 0
#define RX_DATA_FLOAT_NUM 3
#define RX_DATA_LEN 16

extern int MCUHB;

//发数
struct TxMessage
{
  float wheel_1_vel;//轮1速度
  float wheel_2_vel;//轮2速度
  float wheel_3_vel;//轮3速度
};

//收数
struct RxMessage
{
  float MCUWheel_1;//轮1实际速度
  float MCUWheel_2;//轮2实际速度
  float MCUWheel_3;//轮3实际速度
};
//float char转换
union trans_t
{
  float dataf[TX_DATA_FLOAT_NUM];
  char datac[TX_DATA_FLOAT_NUM*4];
};

union trans_r
{
  float dataf[RX_DATA_FLOAT_NUM];
  char datac[RX_DATA_FLOAT_NUM*4];
};

class commSerial
{
private:
    /* data */
    struct termios termAttr;            // 终端标准接口
    struct sigaction saio;
    int ifSerial = 0;
    const char *this_port;              // 串口号
    uint32_t this_baudrate = 9600;      // 波特率

public:
  commSerial(const char *port = "",uint32_t baudrate = B115200);
  ~commSerial();
public:   
  void commSerialInit();              // 串口初始化
  int commSerialRead();               // 串口读取函数
  void commSerialWrite();             // 通过串口发送函数
  int  commSerialIsopen();            // 检查串口是否连接
  void CommunicateRMcu();             // 通信收数主函数
  void CommunicateSMcu(); 	          // 通信发数主函数
	void WaitForMcuPrepared(void);      // 等待mcu

public:
    int handle;                         // 串口文件描述句柄    
    unsigned char txbuf[TX_DATA_LEN];   // 发送数据缓存数组
    unsigned char rxbuf[RX_DATA_LEN];   // 接收数据缓存数组

    TxMessage tx_data;
    RxMessage rx_data;
};

extern commSerial *commserial;

#endif