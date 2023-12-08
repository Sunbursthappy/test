#include <control/debugfile.hpp>
#include <control/path.hpp>

std::ofstream fdebug;
std::ofstream fringbuf;
std::string filename;

//时间转换为字符串类型
std::string Convert(float Num)
{
	std::ostringstream oss;

  //一位数前面补0
  if ((Num/10) < 1)
  {
    oss<<0;
  }

	oss<<Num;
	std::string str = oss.str();
	return str;
}

//生成debug文件
void GetFile()
{
  time_t timeReal;
  tm* t;

  //获取系统时间戳
  time(&timeReal);
  timeReal = timeReal + 8*3600;//	格林尼治标准时间+8个小时
  t = gmtime(&timeReal);

  //生成debug文件夹
  if (NULL == opendir(FILE_PATH)) {
		int r = mkdir(FILE_PATH, 0777);
	}

  //获取debug文件名
  filename = FILE_PATH + Convert(t->tm_year+1900) + "-" + Convert(t->tm_mon + 1) + "-" + Convert(t->tm_mday)
    + "-" + Convert(t->tm_hour) + ":" + Convert(t->tm_min);

  fdebug.open(filename + "debug.txt");
}

//关闭debug文件
void CloseFile()
{
  fdebug.close();
}

//输出到debug文件
void Debug(std::shared_ptr<class controller> node)
{
  std::cout << "Debug" << std::endl;
  fdebug << "wheel_1\t" << node->wheel_1 << "\twheel_2\t" << node->wheel_2 << "\twheel_3\t" <<node->wheel_3
    << "\tposx\t" << node->pos.point.x << "\tposy\t" << node->pos.point.y << "\tangle\t" << node->pos.poseAngle
    << "\tactvel\t" << node->pos.vel << "\ttarvel\t" << node->tarvel.vel << "\ttardirection\t" << node->tarvel.direction
    << "\ttaromega\t" << node->tarvel.omega 
    << "\ttarX\t" << node->tarPos.point.x << "\ttarY\t" << node->tarPos.point.y
    << "\trealPostoVPos\t" << realPostoVPos << "\trealPostoVTar\t" << realPostoVTar
    << "\trealDistance\t" << realdis << "\tadjustDistance\t" << adjustdis
    << "\tTraveled\t" << lengthWheelTraveled << "\tVelCircleDirection\t" << VelCirDir 
    << "\terr\t" << g_err << "\tlengthsum\t" << g_lengthsum << "\twalkstatus\t" << node->walkStatus
    << "\tact1\t" << node->actWheel_1 << "\tact2\t" << node->actWheel_2 << "\tact3\t" << node->actWheel_3
    << std::endl;
}

void OutputRingBuf()
{
  fringbuf.open(filename + "ringbuf.txt");
  std::cout << "CreatRingBuf!" << std::endl;
  for(int i=0;i<RINGBUFSIZE;i++)
  {
    fringbuf << "x=\t" << ringBuffer[i].point.x <<"\ty=\t" << ringBuffer[i].point.y << "\tposangle=\t" << ringBuffer[i].poseAngle
      << "\tvelangle=\t" << ringBuffer[i].angle << "\tvel=\t" << ringBuffer[i].vellMax 
      << std::endl;
  }
}