#ifndef _BSPLINE_HPP_
#define _BSPLINE_HPP_

#include <control/path.hpp>

float CaculateBsplineLen(Point_t point1, Point_t point2, float angle1, float angle2);
PointU_t SerchBsplineVirtualPoint(Point_t point1, Point_t point2, float angle1, float angle2, float robotlen);
PointU_t SerchVirtualPoint(float robotLen);
PointU_t SerchVirtualPoint2(float robotLen);
int BspSegment(int num, Pose_t* points, KeyPointInf_t* keyPoint);

//在线规划信息结构体
typedef struct
{
    //轨迹x
	double xPara[6][6];//第一维是第几段轨迹，第二维是x方向的留个轨迹方程系数
	double yPara[6][6];//第一维是第几段轨迹，第二维是y方向的留个轨迹方程系数
	double segTime[6];//分段时间比例
	double xPos[10];//x位置点
	double yPos[10];//y位置点
	//速度值可以给小，轨迹的速度方向约束可以认为给定
	double xVel[10];//x速度大小
	double yVel[10];//y速度大小
	double miniTotalTime;//轨迹总时间
	float poseAngle;//姿态角
	int segNum;//轨迹段数
	double vel[10];
	double direction[10];
}robotMinSnap_t;


#endif



