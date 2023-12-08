#ifndef _PATHFOLLOWING_HPP_
#define _PATHFOLLOWING_HPP_

#define DEBUG

#include <control/path.hpp>
#include <control/timer.hpp>
#include <control/ring.hpp>
#include <rclcpp/rclcpp.hpp>

#define WaitForStart    (0)
#define GoForPath       (1)
#define STOP            (2)

#define NO_PLANNING  (0)
#define IS_PLANNING  (1)

#define PERCENT_PATH  (0.5)

#define RINGBUFSIZE (400)

extern int realPostoVPos;
extern int realPostoVTar;
extern int VelCirDir;
extern int adjustdis;
extern int realdis;

void AngleLimit(float *angle);
float CalculateLineAngle(Point_t pointStart, Point_t pointEnd);
float CalculatePoint2PointDistance(Point_t point1, Point_t point2);
float CalculateAngleAdd(float angle1, float angle2);
float CalculateAngleSub(float minuend, float subtrahend);


class controller : public rclcpp::Node
{
public:
  controller();

  void SysInit();

  Pose_t pos,lastpos;
  gBlock_t gBlock;
  Vel_t tarvel;
  PointU_t tarPos;
  float actVelx,actVely;
  float wheel_1,wheel_2,wheel_3;
  float actWheel_1,actWheel_2,actWheel_3;
  int walkStatus;
  int lastWalkStatus;

  int Pathfollowing(float percent);
  void Walk();
  void CalculateWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree);
  void SpeedPlanning(void);
  int InputPoints2RingBuffer(Pose_t *points,int num);
  void VelControl(Vel_t actVel);
  void VelCtrlTask(void);

  void GetTarWheel(float vel,float direction,float omega);
  Vel_t GetAdjustVel(Point_t robotPos,PointU_t adjustTarget,float vell);
  void AdjustVel(float *carVel,float *velAngle,Vel_t adjustVel);
  float AngleControl(float anglePresent,float angleTarget,float kp,float kd);
  float PostionControl(float distance, float kp, float kd);

  //订阅定位系统信息
  rclcpp::Subscription<base_interface::msg::Ops>::SharedPtr sub;
  void RxCallBack(base_interface::msg::Ops::SharedPtr RxMsg);
};

#endif