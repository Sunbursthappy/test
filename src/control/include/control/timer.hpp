#ifndef __TIME_HPP_
#define __TIME_HPP_

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <signal.h>

//等待时间，单位是s
#define WAITTIME 1    //1s等待时间

//间隔时间，单位是us
#define INTERVAL 5000 //5ms间隔时间

extern int timeFlag_5ms;//5ms标志
extern int timeFlag_10ms;//10ms标志

void TimeInit();
void signal_Time_5ms(int sig_num);//signal函数调用，要求必须有int类型的形参

#endif