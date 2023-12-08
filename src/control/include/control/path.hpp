#ifndef _PATH_HPP_
#define _PATH_HPP_

#include <rclcpp/rclcpp.hpp>
#include <base_interface/msg/ops.hpp>
#include <memory>
#include <control/com.hpp>
#include <control/timer.hpp>
#include <cmath>

#define PI             (3.14159265)
#define TARGET_VEL     (300.0)
#define MAX_TAR_VEL    (10000.0)
#define MAX_TAR_ACC    (7000.0)
#define MAX_ADJUST_VEL (1000.0)

#define SMALL_VELL     (150.f)

//底盘旋转半径
#define MOVEBASE_RADIUS (250.0f)

#define ADJUST_KP      (1.0f)

#define CIRCLE_PATH_NUM_DEF (102)

//1轮与中心连线切线方向  逆时针为正
#define ONE_VERTICAL_ANG (60.0)
//2轮与中心连线切线方向
#define TWO_VERTICAL_ANG (-60.0)
//3轮与中心连线切线方向
#define THREE_VERTICAL_ANG (180.0)

extern float lengthWheelTraveled;
extern float g_err;

typedef struct
{
	//取快路径规划次数
	int TimeCnt;
	//取块段终点速度方向
	float Direction;
	float PosX;
	float PosY;
	float AnglePosX;
	float AnglePosY;
	float Angle;
} gBlock_t;

typedef struct
{
	//速度大小
	float vel;
	//速度方向
	float direction;
	//角速度大小
	float omega;
} Vel_t;

typedef struct
{
  float x,y;
} Point_t;

//三轮控制姿态参数
typedef struct
{
	Point_t  point;
	float    angle;
	float    poseAngle;
	float    length;
	float    curvatureR;
	float    vellMax;
	float    accT;
} KeyPointInf_t;

typedef struct
{
	Point_t point;
	//角度制
	float poseAngle;
	//速度
	float vel;
} Pose_t;

extern Pose_t g_pos;

typedef struct
{
  Point_t point;
  float u;
	float direction;
	unsigned short startPtr;
	unsigned short endPtr;
} PointU_t;

extern Pose_t circle[];

struct ExpertPID_t
{
    float setpoint;   	  	/*设定值*/
    float kp;             	/*比例系数*/
    float ki;          		  /*积分系数*/
    float kd;             	/*微分系数*/
    float thisErr;          /*偏差*/
    float lastErr;        	/*前一拍偏差*/
    float preErr;           /*前两拍偏差*/
    float dErr;             /*偏差差值（微分）*/ 
    float lastdErr;         /*上一次偏差差值*/ 
    float result;           /*PID控制器结果*/
    float maximum;          /*输出值上限*/
    float minimum;          /*输出值下限*/
    float errabsmax;        /*偏差绝对值最大值*/
    float errabsmid;        /*偏差绝对值中位值*/
    float errabsmin;        /*偏差绝对值最小值*/
};

class vecvel
{
public:
  float x,y;
  float vel;
  float direction;
public:
  vecvel();
  void GetModule();
  void TransModule();
  vecvel operator+(const vecvel &t) const;
  vecvel operator-(const vecvel &t) const;
};

Point_t Getpps(Pose_t pos);


extern _Bool CaculateLenFlag;

void GetVel(float actVelx,float actVely);

void CaculatePath(void);

void UpdateLenBegin(void);

void UpdateLenStop(void);

void AddPath(float dis);

void ReducePath(float dis);

float GetPath(void);

void ClearPathLen(void);

float Get_AverLength(float ppsx, float ppsy);

Vel_t CalcWheelSpeed(float vel, float direction, float omega, int wheelNum, float postureAngle);

#endif