#include <control/timer.hpp>

//5ms定时器标志
int timeFlag_5ms;
//5ms定时器计数
int cnt;
//10ms定时器标志
int timeFlag_10ms;

//定时器标志改变
void signal_Time_5ms(int sig_num)
{
	timeFlag_5ms = 1;
	cnt++;
	if(cnt == 2)
	{
		cnt = 0;
		timeFlag_10ms = 1;
	}
}

//定时器初始化
void TimeInit()
{
	signal(SIGALRM,signal_Time_5ms);     //signal调用的函数定义要写在前面
	struct itimerval itv;
 
 //计时器第一次启动时间
	itv.it_value.tv_sec     = WAITTIME;  //单位：秒
	itv.it_value.tv_usec    = 0;         //单位：微秒
 
 //计时器重启动的间歇值
	itv.it_interval.tv_sec  = 0;				 //单位：秒
	itv.it_interval.tv_usec = INTERVAL;  //单位：微秒

	setitimer(ITIMER_REAL,&itv,NULL);
}