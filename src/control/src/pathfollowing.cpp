#include <control/pathfollowing.hpp>
#include <control/Bspline.hpp>
#include <control/timer.hpp>
#include <control/com.hpp>
#include <control/debugfile.hpp>

/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
* @note
* @param  angle:要限制的值
* @retval 
*/
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;

	recursiveTimes++;

	if (recursiveTimes < 100)
	{
		if (*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if (*angle < -180.0f)
		{
			*angle += 360.0f;
			AngleLimit(angle);
		}
	}

	recursiveTimes--;
}

float CalculateLineAngle(Point_t pointStart, Point_t pointEnd)
{
	float a = 0.0f;
	float b = 0.0f;

	a = pointEnd.y - pointStart.y;
	b = pointEnd.x - pointStart.x;

	return (atan2(a, b)/PI*180);
}

float CalculatePoint2PointDistance(Point_t point1, Point_t point2)
{
	float dis;
	dis = sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	return dis;
}

float CalculateAngleAdd(float angle1, float angle2)
{
	float result = 0.0f;
	result = angle1 + angle2;
	if (result >  180.0f)  result -= 360.0f;
	if (result < -180.0f)  result += 360.0f;
	return result;
}

float CalculateAngleSub(float minuend, float subtrahend)
{
	float result = 0.0f;
	result = minuend - subtrahend;
	if (result >  180.0f)  {result -= 360.0f;}
	if (result < -180.0f)  {result += 360.0f;}
	return result;
}

//实体化
controller::controller() : Node("controller")
{
  sub = this->create_subscription<base_interface::msg::Ops>("/ops_data", 10, 
    std::bind(&controller::RxCallBack, this, std::placeholders::_1));
}

void controller::SysInit()
{
	std::cout << "Waiting for Init" << std::endl;
#ifndef DEBUG
  commserial->commSerialInit();
  commserial->WaitForMcuPrepared();
#endif
	BufferZizeInit(RINGBUFSIZE);
  TimeInit();
  GetFile();
	walkStatus=0;
	lastWalkStatus=0;
	std::cout << "Init Finished" << std::endl;
}

//计算每个轮子速度
void controller::GetTarWheel(float vel,float direction,float omega)
{
	wheel_3 = vel *cos(PI/180*(180.0 - direction + pos.poseAngle)) + PI/180*(omega) * MOVEBASE_RADIUS;
	wheel_2 = vel *cos(PI/180*(60.0  + direction - pos.poseAngle)) + PI/180*(omega) * MOVEBASE_RADIUS;
	wheel_1 = vel *cos(PI/180*(60.0  - direction + pos.poseAngle)) + PI/180*(omega) * MOVEBASE_RADIUS;
}

int realPostoVPos;
int realPostoVTar;
int VelCirDir;
int controller::Pathfollowing(float percent)
{
  static uint8_t angleAdapt = 0;
	static uint8_t cvAdpet = 0;
	static float vell = 150.0f;
	static uint8_t recordTime = 0;
	static uint8_t edgeBlock2Taki = 0;
	static uint8_t edgeCnt = 0;
	static uint8_t takiJudge = 0;
	static uint8_t firstStrikeCnt = 0;
	static float lastAngulatVel = 0.f;
	static Point_t firstStrikePos;
	float velDir = 0.0f;
	float angularVel = 0.0f;
	float angleErr = 0.0f;
	float posAngleVP = 0.0f;
	float posAngleVT = 0.0f;
	float robotlen = 0.0f;
	float disRealPos2VirTarget = 0.0f;
	float disRealPos2VirPos = 0.0f;
	float poseAngleArugle = 0.f;
	Vel_t adjustVel = {0.0f};
	PointU_t virtualPos,virtualTarget;
	PointU_t stopPoint;
	stopPoint.point.x =  676.703f;
	stopPoint.point.y = 9650.000f;
	stopPoint.direction = 0;

	if(percent < 0.0f || percent > 1.2f)
	{
		printf("Invalid parameter\n");
		return -1;
	}

	//当前点与虚拟位置点距离 和 虚拟位置点到虚拟目标点距离之和
	float VIEW_L = 0.0f;
	if (!lastWalkStatus)
	{
		recordTime = 0;
		angleAdapt = 0;
		edgeBlock2Taki = 0;
		cvAdpet = 0;
		takiJudge = 0;
		firstStrikeCnt = 0;
	}
	if (recordTime == 0)//开始记录长度
	{
		UpdateLenBegin();
		recordTime = 1;
	}

	switch(walkStatus)
	{
		case WaitForStart:
		{
			VIEW_L = pos.vel/100.0f*10.f;
			break;
		}
		case GoForPath:
		{
			VIEW_L = pos.vel/100.0f*10.f*1.8f;
			break;
		}
		default:
		{
			VIEW_L = pos.vel/100.0f*10.f;
			break;
		}
	}
	//获取定位系统所计算的机器人实际行走路径长度
	robotlen = GetPath();
	
	//虚拟位置点
	virtualPos = SerchVirtualPoint(robotlen);
	tarPos = virtualPos;
	
	//计算当前点到虚拟位置点的距离(直线距离)	
	disRealPos2VirPos = CalculatePoint2PointDistance(pos.point,virtualPos.point);
	
	realPostoVPos = disRealPos2VirPos;

	if(VIEW_L - disRealPos2VirPos >= 0.0f)
	{
		//加上 提前量与偏移量的差值
		robotlen = GetPath() + VIEW_L - disRealPos2VirPos;
	}

	//求取虚拟目标点
	virtualTarget = SerchVirtualPoint2(robotlen);
	
	//计算实际位置距离虚拟目标点距离
	disRealPos2VirTarget = CalculatePoint2PointDistance(pos.point,virtualTarget.point);

	realPostoVTar = disRealPos2VirTarget;

	float disAdd = (VIEW_L - disRealPos2VirPos) - disRealPos2VirTarget;
	if(GetPath() < GetLength())
	{
		if(disAdd > 0.f)
		{
			AddPath(2.f*disAdd);
		}
	}
	else
	{
		//当记录的路程大于轨迹总路程后停止记录路程
		UpdateLenStop();
	}
	
	//虚拟位置点姿态角
	posAngleVP = CalculateAngleAdd(GetRingBufferPointPoseAngle(1),CalculateAngleSub(GetRingBufferPointPoseAngle(2) , GetRingBufferPointPoseAngle(1))*virtualPos.u);
	
	//两端点之间角度的插值
	angleErr = CalculateAngleSub(GetRingBufferPointPoseAngle(virtualTarget.endPtr) , GetRingBufferPointPoseAngle(virtualTarget.startPtr));
	
	//虚拟目标点姿态角
	posAngleVT = CalculateAngleAdd(GetRingBufferPointPoseAngle(virtualTarget.startPtr),angleErr*virtualTarget.u);
	
	
	//角速度
	//相对于视觉坐标的角度
	static float cvBlockAngle = 0.f;
	//用视觉角度调整姿态角标志位
	static uint8_t cvAngleFlag = 0;
	//雷达未识别到的次数
	static uint8_t lidarNotCnt = 0;
	
	switch(walkStatus)
	{
		case WaitForStart:
		{
			angularVel = AngleControl(pos.poseAngle,posAngleVT,2.4f,1.8f);
			break;
		}
		case STOP:
		{
			angularVel = AngleControl(pos.poseAngle,stopPoint.direction,2.4f,1.8f);					
			break;
		}
		default:
		{
			angularVel = AngleControl(pos.poseAngle,posAngleVT,6.f,15.f);					
			break;
		}
	}
	lastAngulatVel = angularVel;

	//目标速度方向
	velDir = virtualTarget.direction;
	
	AngleLimit(&velDir);
	
	//目标速度
	vell = GetRingBufferPointVell(virtualTarget.startPtr)+(GetRingBufferPointVell(virtualTarget.endPtr) - GetRingBufferPointVell(virtualTarget.startPtr))*virtualTarget.u;

	vell = vell*percent;

	{
		if(disRealPos2VirPos>50.0f && disRealPos2VirPos < 100.0f)
		{
			if(vell*vell > 2.0f*(0.5f*MAX_TAR_ACC)*(disRealPos2VirPos - 50.0f))
			{			
				vell = vell*(1-0.006f*(disRealPos2VirPos - 50.f));
			}
			else
			{
				vell = 0.7f*vell;
			}
		}
		else if(disRealPos2VirPos >= 100.0f)
		{
			if(vell*vell > 2.0f*(0.5f*MAX_TAR_ACC)*(disRealPos2VirPos - 100.0f))
			{		
				vell = 0.7f*vell;
			}
			else
			{
				vell = 0.8f*vell;
			}
		}
	}

	if(walkStatus != STOP)
	{
		//计算当前点到位置点的位置调节量（速度环用到虚拟目标点）
		adjustVel = GetAdjustVel(pos.point,virtualPos,vell);
		VelCirDir = velDir;
		//将调节速度和规划的速度进行合成
		AdjustVel(&vell,&velDir,adjustVel);
		tarvel.vel=vell;
		tarvel.direction = velDir;
		tarvel.omega = angularVel;
	}
	else
	{
		adjustVel = GetAdjustVel(pos.point,stopPoint,500);
		tarvel.vel = adjustVel.vel;
		tarvel.direction = adjustVel.direction;
		tarvel.omega = angularVel;
	}
}

/*********************************************************************************
* @name 	AngleControl
* @brief	角度闭环控制程序
* @param	anglePresent 当前的角度 单位 度
* @param  angleTarget  目标角度   单位 度
* @retval	无
**********************************************************************************/
float controller::AngleControl(float anglePresent,float angleTarget,float kp,float kd)
{
	/****************************普通PD控制*******************************/
	
	float angleErr = 0.0f,angularVel = 0.0f, angularVelErr = 0.0f;
	static float lastAngleErr = 0.0f, preAngleErr = 0.0f, lastAngledTerm = 0.0f;
	float dTerm = 0.0f,dTermFliter = 0.0f;

	//PD控制器
	//目标角度减去当前角度
	angleErr = CalculateAngleSub(angleTarget,anglePresent);
	static float lastCvAngle = 0.f;
	static float lastAngleOutPut = 190.f;
	static uint8_t  timeCnt = 0;

	//换点清零
	if (!lastWalkStatus)
	{
		timeCnt = 0;
		lastCvAngle = 0.f;
	}
	
	dTerm = (angleErr - lastAngleErr);
	if(fabs(dTerm) > 30.f)
	{
		dTermFliter = 0.2f*dTerm + 0.8f*lastAngledTerm;
		angularVel = angleErr * kp * 0.6f + dTermFliter * kd;
	}
	else
	{
		//低通滤波
		dTermFliter = 0.5f*dTerm + 0.5f*lastAngledTerm;
		angularVel = angleErr * kp + dTermFliter * kd;
	}
	
	lastAngledTerm = dTerm;
	lastAngleErr = angleErr;
	preAngleErr = lastAngleErr;

	angularVel = angularVel + angularVelErr *0.1f;

	//防止输出突变
	if (fabs(angularVel) > fabs(lastAngleOutPut) * 1.2f && (fabs(lastAngleOutPut) > 20.f || lastAngleOutPut * angularVel < 0.f))
	{
		angularVel = fabs(angularVel)/angularVel * fabs(lastAngleOutPut) * 1.2f;
	}
	if (fabs(lastAngleOutPut - angularVel) > 35.f)
	{
		angularVel = (lastAngleOutPut + angularVel)/2.f;
	}
	
	lastAngleOutPut = angularVel;
	return angularVel;
} 
/*********************************************************************************
* @name 	PostionControl
* @brief	位置闭环控制程序
* @param	distance 当前的位置误差 单位 mm
* @param  	kp 专家PID参数
* @param  	kd 专家PID参数

* @retval	无
**********************************************************************************/
float controller::PostionControl(float distance, float kp, float kd)
{
	/********************************专家PID****************************
	 *    5条规则
	 * 1. 误差绝对值很大，按照最大（绝对值）输出
	 * 2. 误差×delta误差>0，误差增大，如果误差值大于中值，则加大控制力度，否则一般控制
	 * 3. 误差×delta误差<0 && delta误差×delta（上一次误差）>0，误差减小 ，一般控制
	 * 4. 误差×delta误差<0 && delta误差×delta（上一次误差）<0，处于极值 ，一般控制（保持输出不变即可）
	 * 5. 误差很小，一般控制（可能是系统静差）
	 */

	static ExpertPID_t distancePID;
  float adjustVelOutput = 0.0f, adjustVelOutputErr = 0.0f;//本次调节输出值
	
	distancePID.maximum = 1500.f;//最大输出速度 待修正
	distancePID.minimum = 0.f;//待修正
	distancePID.errabsmax = 400.f;// 最大距离误差 
	distancePID.errabsmid = 225.f;//中等距离误差 
	distancePID.errabsmin =  46.f;//小距离误差   

	distancePID.kp = kp;
	distancePID.kd = kd;
	distancePID.thisErr = distance;//本次偏差
	distancePID.dErr = distancePID.thisErr - distancePID.lastErr;//偏差微分

	if(fabs(distancePID.thisErr) > distancePID.errabsmax)//误差太大，执行规则1
	{
		distancePID.result = fabs(distancePID.thisErr) / distancePID.thisErr * distancePID.maximum;//最大输出
	}	
	else if((distancePID.thisErr * distancePID.dErr > 0)||(fabs(distancePID.dErr) < 0.1f))//误差增大，执行规则2
	{
		if(fabs(distancePID.thisErr) >= distancePID.errabsmid)//误差较大,加大控制
		{
			distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
		}
		else//误差较小，一般控制
		{
			distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
		}
	}
 	else if(((distancePID.thisErr * distancePID.dErr < 0) && (distancePID.dErr * distancePID.lastdErr > 0)) || (distancePID.thisErr == 0))//误差减小，执行规则3，一般控制
  	{
    	distancePID.result = 1.0*(distancePID.thisErr * distancePID.kp + distancePID.dErr * distancePID.kd);
  	}
	else if((distancePID.thisErr * distancePID.dErr < 0) && (distancePID.dErr * distancePID.lastdErr < 0))//误差极值，执行规则4
  	{
    	if(abs(distancePID.thisErr) >= distancePID.errabsmid) //误差较大，则较强控制
		{
			distancePID.result = 1.0* distancePID.kp * distancePID.thisErr;
		}
		else//误差一般大，一般控制
		{
			distancePID.result = 1.0 * distancePID.kp * distancePID.thisErr;
		}
  	}
	else if((fabs(distancePID.thisErr) <= distancePID.errabsmin) && (abs(distancePID.thisErr)>0))//误差很小，执行规则5
	{
		distancePID.result = 1.0 * distancePID.kp * distancePID.dErr + 0.8 * distancePID.ki * distancePID.thisErr;//可能存在稳态误差，添加ki控制
	}
	distancePID.preErr = distancePID.lastErr;
	distancePID.lastErr = distancePID.thisErr;
	distancePID.lastdErr = distancePID.dErr;

	return distancePID.result;
	/********************************专家PID***************************/
}

int realdis;
int adjustdis;
Vel_t controller::GetAdjustVel(Point_t robotPos,PointU_t adjustTarget,float vell)
{
  Vel_t adjustVel = {0};
	float distance = 0.0f;
	float angle = 0.0f;

  angle = CalculateLineAngle(robotPos,adjustTarget.point);

  distance = CalculatePoint2PointDistance(robotPos,adjustTarget.point);
	realdis = distance;

  if(distance<=0.0f)
	{
		distance = 0.0f;
	}
	//计算调节速度大小和方向

	if(distance <= 20.0f)
	{
		distance = distance * 0.8f;
	}
	else if(distance <= 40.0f) 
	{
		distance = distance * 1.0f - 4.0f;
	}
	else if(distance <= 60.0f)
	{
		distance = distance * 1.4f - 20.f;
	}
	else if(distance <= 100.0f)
	{
		distance = distance * 2.0f - 56.f;
	}
	else
	{
		distance = distance * 2.4f - 96.f;
	}
	adjustdis = distance;

  adjustVel.vel = distance * 4.5f;

  if (vell < 300.f)
	{
		adjustVel.vel = adjustVel.vel * 0.5f;
	}
	else if(vell < 500.f)
	{
		adjustVel.vel = adjustVel.vel * 0.6f;
	}
  adjustVel.direction = angle;
	
	//对调节速度大小进行限制
	if(adjustVel.vel>=MAX_ADJUST_VEL)
	{
		adjustVel.vel = MAX_ADJUST_VEL;
	}
	
	return adjustVel;
}

void controller::AdjustVel(float *carVel, float *velAngle, Vel_t adjustVel)
{
  float angleErr = 0.0f;
	float projectOnvel = 0.0f;
	vecvel oriVel,adjust,result;
	
	oriVel.vel = *carVel;
	oriVel.direction = *velAngle;
  oriVel.TransModule();
	
	//计算调节目标速度的大小
	adjust.vel = ADJUST_KP * adjustVel.vel;
	
	//计算速度方向的调节量并进行限制
	angleErr = adjustVel.direction - *velAngle;
	angleErr = angleErr > 180.0f ? angleErr - 360.0f : angleErr; 
	angleErr = angleErr < -180.0f ? 360.0f + angleErr : angleErr;

	angleErr =  ADJUST_KP * angleErr;
	if (angleErr > 180.0f)
	{
		angleErr = 180.0f;
	}
	else if (angleErr < -180.0f)
	{
		angleErr = -180.0f;
	}
	
	adjust.direction = *velAngle + angleErr;
	
	adjust.direction = adjust.direction > 180.0f ? adjust.direction - 360.0f : adjust.direction; 
	adjust.direction = adjust.direction < -180.0f ? 360.0f + adjust.direction : adjust.direction;
  adjust.TransModule();
	
	result = oriVel + adjust;

	//对调整结果的速度大小进行限制
	if(result.vel >= MAX_TAR_VEL)
	{
		result.vel = MAX_TAR_VEL;
    result.TransModule();
	}
	
	*carVel   = result.vel;
	*velAngle = result.direction;
}

void controller::Walk(void)
{
	
	/****************************************MCU********************************************/
	//记录case
	static int walkCnt = 0;
	switch (walkStatus)
	{
		case WaitForStart: 
		{
			float initVelAngle = 0;
			initVelAngle = pos.poseAngle;
			AngleLimit(&initVelAngle);
			GetTarWheel(0.0f,initVelAngle,0.0f);

			std::cout << "Planning" << std::endl;

			ClearRingBuffer();
		
			//规划第一段轨迹
			circle[0].point.x = pos.point.x;
			circle[0].point.y = pos.point.y;
			circle[0].poseAngle = pos.poseAngle;
			InputPoints2RingBuffer(circle, CIRCLE_PATH_NUM_DEF);
			OutputRingBuf();

			std::cout << "PlanningFinished" << std::endl;

			walkStatus = GoForPath;
			lastWalkStatus = WaitForStart;
		
			break;
		}

		case GoForPath:
		{
			float dis2FinalAngle = pos.poseAngle - circle[CIRCLE_PATH_NUM_DEF -1].poseAngle;
			AngleLimit(&dis2FinalAngle);

			//跟随轨迹
			Pathfollowing(PERCENT_PATH);

			if (lengthWheelTraveled >= g_lengthsum)
			{
				walkStatus = STOP;
			}

			lastWalkStatus = GoForPath;
			break;
		}

		case STOP:
		{
			//跟随轨迹
			Pathfollowing(1.0f);
			if ((pos.point.x-(-500.000) >= -10) && (pos.point.x-(-500.000) <= 10)
				&& (pos.point.y-(9650.000) >= -10) && (pos.point.y-(9650.000) <= 10))
			{
				GetTarWheel(0,0,0);
			}
			else
			{
				GetTarWheel(0.3*tarvel.vel,tarvel.direction,tarvel.omega);
			}
			
			lastWalkStatus = STOP;
			break;
		}
	}
}

//传入点信息，进行规划。并放入缓存池
int controller::InputPoints2RingBuffer(Pose_t *points,int num)
{
	int n = 0;
	//将传入的点绘制为B样条
	n = BspSegment(num,points,GetRingBufferAdress());
	//记录填入的关键点个数
	SetUpPointer(n);
	//对第一点做处理 设置第一点的据起点长度为0
	SetRingBufferPointLen(1,0.0f);
	for(int i = 1;i < n;i++)
	{
		//求出本段曲线的曲线长度
		float tempLenth = CaculateBsplineLen(GetRingBufferPoint(i),GetRingBufferPoint(i + 1),GetRingBufferPointAngle(i),GetRingBufferPointAngle(i+1));
		//设置该点距离起点路径长度
		SetRingBufferPointLen(i+1 ,GetLength() + tempLenth);
		//设置总长度
		SetLength(GetLength() + tempLenth);
		//曲率半径 R = L / θ 
		float curvatureR = 0.0f;
		if(fabs(CalculateAngleSub(GetRingBufferPointAngle(i+1),GetRingBufferPointAngle(i))) < 0.01f)
		{
			//直线的曲率半径
		  curvatureR = fabs(tempLenth)/0.0001f;
		}
		else
		{
			//曲线的曲率半径
			curvatureR = fabs(tempLenth/((CalculateAngleSub(GetRingBufferPointAngle(i+1),GetRingBufferPointAngle(i)))/180*PI));
		}
		//设置该点曲率半径
		SetRingBufferAverCurvature(i,curvatureR);
	}

	
	//将存入的数据规划速度
	SpeedPlanning();

	//清除机器人行走路径长度

	ClearPathLen();
	return 1;
}

//通过ringBuffer里的数据计算每一点处三个轮子的速度
//目的更新wheelOne wheelTwo wheelThree这三个数组里的三轮速度，便于下一次的速度削减
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void controller::CalculateWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree)
{
	//分解到三个轮对全局速度进行规划
	float n = GetCount();
	for (int i = 2; i < n + 1; i++)
	{
		float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;

		float time = 0.0f;

		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

		float rotationVell = angErr / time;

		wheelOne[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
										1 , GetRingBufferPointPoseAngle(i)).vel;

		wheelTwo[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
										2 , GetRingBufferPointPoseAngle(i)).vel;

		wheelThree[i - 1] = CalcWheelSpeed(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell,\
										3 , GetRingBufferPointPoseAngle(i)).vel;

	}
	wheelOne[0] = wheelOne[1];
	wheelTwo[0] = wheelTwo[1];
	wheelThree[0] = wheelThree[1];
}

#define MIN_VELL (150.0f)
//速度规划函数
float testPlanVel = 0.0f;
void controller::SpeedPlanning(void)
{
	int n = GetCount();
	int troughCnt = 0; //波谷个数
	int* troughNumRecord=NULL;
	float* vell = NULL;
	float* curvature = NULL;
	float* curveNormalDirection = NULL;	
	float* wheelOne = NULL;
	float* wheelTwo = NULL;
	float* wheelThree = NULL;
	

	troughNumRecord = (int *)malloc(n*sizeof(int));
	vell = (float *)malloc(n*sizeof(float));
	curvature = (float *)malloc(n*sizeof(float));	   
	curveNormalDirection = (float *)malloc(n*sizeof(float));
	wheelOne = (float *)malloc(n*sizeof(float));
	wheelTwo = (float *)malloc(n*sizeof(float));
	wheelThree = (float *)malloc(n*sizeof(float));

	//记录每一个点的曲率半径并计算每个点的法向方向
	for (int i = 0; i < n; i++)
	{ 
		curvature[i] = GetRingBufferAverCurvature(i + 1);
		curveNormalDirection[i] = GetRingBufferPointAngle(i + 1) + 90.0f;
		AngleLimit(&curveNormalDirection[i]);
	}
	//为曲率半径起止位置赋值
	curvature[0] = curvature[1];
	curvature[n - 1] = curvature[n - 2];
	for (int i = 1; i < n - 1; i++) //首尾不算波谷
	{
		if(curvature[i] <= curvature[i+1] && curvature[i] <= curvature[i-1])
		{
			troughNumRecord[troughCnt] = i;//记录的是波谷的下标
			troughCnt ++;//记录波谷的个数
		}
	}
	for(int j = 0;j < 1;j++ )
	{
		for (int i = 1; i < n-2; i++)
		{
			if(curvature[i] < 20000 &&curvature[i+1] < 20000 &&curvature[i-1] < 20000)
			{
				if(curvature[i] >= curvature[i+1] && curvature[i] >= curvature[i-1])
				{
					curvature[i] = (curvature[i+1]+curvature[i-1])/2;
				}
			} 
		}
	}
	for(int i = 0;i < troughCnt - 1;i ++)//遍历所有的波谷（曲率半径最小点）,除了最后一个点
	{
		int cutFlag = 1;
		for(int j = (troughNumRecord[i]+1);j < troughNumRecord[i+1];j ++)//遍历和下一个波谷之间的所有点
		{
			if(curvature[j] > 5000)//存在曲率半径很大的点
				cutFlag = 0;
		}
		if(cutFlag == 1)//两波谷之间无大于5000的点
		{
			for(int j = (troughNumRecord[i]+1);j < troughNumRecord[i+1];j++)//遍历和下一个波谷之间的所有点,平滑线性化
			{
				curvature[j] = curvature[j-1] + (curvature[troughNumRecord[i+1]] - curvature[troughNumRecord[i]])/(troughNumRecord[i+1]-troughNumRecord[i]);
			}
		}
	}
	
	//通过曲率半径计算该段能满足的最大速度
	for (int i = 0; i < n; i++)
	{
		vell[i] = sqrt((1.0f * MAX_TAR_ACC) * curvature[i]);
		if(vell[i] > MAX_TAR_VEL)
		{
			vell[i] = MAX_TAR_VEL;
		}
	}

	//将初始速度和终止速度改为给定值  与传入变量 vellMax比较
	for (int i = 0; i < n; i++)
	{
		if(vell[i]>GetRingBufferPointVell(i+1))
		{
			vell[i] = GetRingBufferPointVell(i+1);
		}
	}
	switch (walkStatus)
	{
		case WaitForStart:
		{
			if (vell[0] < 500.f/PERCENT_PATH)
			{
				vell[0] = 500.f/PERCENT_PATH;
			}
			break;
		}
		default:
		{
			if (vell[0] < 500.f)
			{
				vell[0] = 500.f;
			}
		}
	}
	//临时计算速度变量
	float tempVell = 0.0f;
	//临时计算该段轨迹结束的最大速度
	float tempVirtualVel = 0.0f;
	//估计的该段轨迹平均速度
	float tempTargetVel = 0.0f;
	//法向加速度
	float accN = 0.0f;
	//切向加速度
	float accT = 0.0f;
	//法向加速度方向
	float accNAngle = 0.0f;
	//估计每段轨迹的速度方向
	float tempAngle = 0.0f;
	//速度变化方向
	float angleChange = 0.0f;
	//留有的加速度余量
	#define ACC_REDUNDANT (0.05f)
	//通过v2^2 - v1^2 = 2*a*s对速度再次规划
	for (int i = 0; i < n - 1; i++)
	{
		//加速过程
		if (vell[i + 1] > vell[i])
		{
			//计算两点间的角度平均值
			tempAngle = (GetRingBufferPointAngle(i + 1) + GetRingBufferPointAngle(i + 2));
			//计算速度方向的变化
			angleChange = GetRingBufferPointAngle(i + 2) - GetRingBufferPointAngle(i + 1);
			if(fabs(angleChange)>180.0f)
			{
				tempAngle+=360.0f;
			}
			tempAngle/=2.0f;
			
			AngleLimit(&tempAngle);
			
			AngleLimit(&angleChange);
			
			//根据速度方向变化计算切向加速度方向
			if(angleChange>=0.0f)
			{
				accNAngle = tempAngle + 90.0f;			
			}
			else
			{
				accNAngle = tempAngle - 90.0f;
			}
			AngleLimit(&accNAngle);
			//估算该段轨迹结束时的速度替代之前的最大速度 v2^2 - v1^2 = 2*a*s
			//tempVirtualVel = sqrt(2 * GetAccLimit(GetRingBufferPoint(i + 1).x,GetRingBufferPoint(i + 1).y) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			tempVirtualVel = sqrt(2 * MAX_TAR_ACC * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);

			//该段轨迹结束的最大速度
			tempTargetVel = vell[i+1];
			//如果估算速度小于最大速度，用估算速度来估算法向加速度
			if(tempVirtualVel<vell[i+1])
			{
				tempTargetVel = tempVirtualVel;
			}
			//计算法向加速度
			accN = pow(tempTargetVel + vell[i],2)/(2.0f * (curvature[i] + curvature[i + 1]));
			//法向加速度较小时忽略不计
			if(accN<=100.0f)
			{
				accN = 0.0f;
			}
			//计算切向加速度
			if(accN< MAX_TAR_ACC)
			{
				accT = sqrtf(MAX_TAR_ACC*MAX_TAR_ACC - accN*accN);
			}
			else
			{
				accT = 0.0f;
			}	
			//根据切向加速度估计该段轨迹结束时的速度 v2^2 - v1^2 = 2*a*s
			tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			if (tempVell < vell[i + 1])
			{
				vell[i + 1] = tempVell;
			}
		}
	}
	
	for (int i = n - 1; i > 0; i--)
	{
		//减速过程
		if (vell[i - 1] > vell[i])
		{		
			//计算两点间的角度平均值
			tempAngle = (GetRingBufferPointAngle(i) + GetRingBufferPointAngle(i + 1));
			//计算速度方向的变化
			angleChange = GetRingBufferPointAngle(i + 1) - GetRingBufferPointAngle(i);
			
			if(fabs(angleChange)>180.0f)
			{
				tempAngle+=360.0f;
			}
			
			tempAngle/=2.0f;
			
			AngleLimit(&tempAngle);
			
			AngleLimit(&angleChange);
			
			//根据速度方向变化计算切向加速度方向
			if(angleChange>=0.0f)
			{
				accNAngle = tempAngle + 90.0f;			
			}
			else
			{
				accNAngle = tempAngle - 90.0f;
			}
			
			AngleLimit(&accNAngle);
			
			//估算该段轨迹结束时的速度替代之前的最大速度
			tempVirtualVel = sqrt(2 * MAX_TAR_ACC  * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			//该段轨迹结束的最大速度
			tempTargetVel = vell[i - 1];
			//如果估算速度小于最大速度，用估算速度来估算法向加速度
			if(tempVirtualVel<vell[i - 1])
			{
				tempTargetVel = tempVirtualVel;
			}
			//计算法向加速度
			accN = pow(tempTargetVel + vell[i],2)/(2.0f * (curvature[i] + curvature[i - 1]));
			//法向加速度较小时忽略不计
			if(accN<=100.0f)
			{                                                                       
				accN = 0.0f;
			}
			//计算切向加速度
			if(accN<MAX_TAR_ACC)
			{
				accT = sqrtf(MAX_TAR_ACC*MAX_TAR_ACC - accN*accN);
			}
			else
			{
				accT = 0.0f;
			}
			//根据切向加速度估计该段轨迹结束时的速度
			tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			if (tempVell < vell[i - 1])
			{
				vell[i -1] = tempVell;
			}
		}
	}

	//将暂时规划的速度放入环形数组里
	for (int i = 0; i < n; i++)
	{
		SetRingBufferPointVell(i + 1, vell[i]);
	}
	//计算此时轮的速度
	CalculateWheelVell(wheelOne, wheelTwo, wheelThree);

	uint8_t ipointLast = 0;
	uint8_t ipointErr[10] = {0},ipointErrCount = 0;
	uint8_t dynamicalAjustingTimes = 0;		
	float carPercent  = 0.01f;


	free(curvature);
	free(vell);
	free(curveNormalDirection);
}

uint8_t perTimeFlag = 0;
void controller::VelControl(Vel_t actVel)
{
	Vel_t velErr = {0.0f};
	Vel_t outputVel = {0.0f};
	float velXErr , velYErr = 0.0f;
	float velXOutput , velYOutput = 0.0f;
	float KpX = 0.2f , KpY = 0.2f;
	float expVelX , expVelY = 0.0f;
	float actVelX , actVelY = 0.0f;
	float vxDerm = 0, vyDerm = 0;
	static float lastVxDerm = 0, lastVyDerm = 0;
	float vxdTermFilter, vydTermFilter;
	static float lastVxErr = 0, lastVyErr = 0, lastVxdTerm = 0, lastVydTerm = 0;
	static float lastVelDir = 0;//last output vel direction

	perTimeFlag++;
	
	expVelX = tarvel.vel*cosf(tarvel.direction*PI/180);
	expVelY = tarvel.vel*sinf(tarvel.direction*PI/180);
	
	actVelX = actVel.vel*cosf(actVel.direction*PI/180);
	actVelY = actVel.vel*sinf(actVel.direction*PI/180);
	
	velXErr = expVelX - actVelX;
	velYErr = expVelY - actVelY;

	if(fabs(GetPath() -GetLength()) < 100.f || sqrt(expVelX*expVelX + expVelY*expVelY) < 300.f)
	{	
		KpX = 0.f;
		KpY = 0.f;
	}

	//限幅&死区
	if(fabs(velXErr) >= 700.0f)
	{
		velXErr = velXErr / fabs(velXErr) * 700.0f;
	}
	else
	{
		if(fabs(expVelX) < 10.0f)
		{
			if(fabs(velXErr) <= 8.0f)
			{
				velXErr = 0.0f;
			}
		}
		else if(fabs(velXErr) <= 8.0f)
		{
			velXErr = 0.0f;
		}		
	}

	if(fabs(velYErr)>=700.0f)
	{
		velYErr= velYErr / fabs(velYErr) * 700.0f;
	}
	else
	{
		if(fabs(expVelY)<10.0f)
		{
			if(fabs(velYErr)<=8.0f)
			{
				velYErr = 0.0f;
			}
		}
		else if(fabs(velYErr)<=8.0f)
		{
			velYErr = 0.0f;
		}		
	}

	velXErr*=KpX;
	velYErr*=KpY;

	vxDerm = velXErr - lastVxErr;
	vyDerm = velYErr - lastVyErr;

	vxdTermFilter = 0.4f * vxDerm + 0.6f * lastVxDerm;
	vydTermFilter = 0.4f * vyDerm + 0.6f * lastVyDerm;

	lastVxDerm = vxDerm;
	lastVyDerm = vyDerm;

	lastVxErr = velXErr;
	lastVyErr = velYErr;


	velXErr +=  vxdTermFilter * 0.4f;
	velYErr +=  vydTermFilter * 0.4f;

	velErr.vel = sqrtf(velXErr*velXErr + velYErr*velYErr);
	velErr.direction = atan2f(velYErr, velXErr)*180/PI;
	
	if(velErr.vel>=1500.0f)
	{
		velErr.vel = 1500.0f;
	}
	
	if(velErr.vel<8.0f)
	{
		velErr.vel = 0.0f;
		velErr.direction = tarvel.direction;
	}
	
	velXOutput = velErr.vel*cosf(velErr.direction*PI/180) + tarvel.vel*cosf(tarvel.direction*PI/180);
	velYOutput = velErr.vel*sinf(velErr.direction*PI/180) + tarvel.vel*sinf(tarvel.direction*PI/180);
	
	outputVel.vel = sqrtf(velXOutput*velXOutput + velYOutput*velYOutput);
	outputVel.direction = atan2f(velYOutput , velXOutput)*180/PI;

	//fixme 如果direction to lunzi发生抖动，小目标速度下可直接不闭环
	
	if(outputVel.vel<180.0f)
	{
		float velAdd = 0.f;
		velAdd = outputVel.direction - lastVelDir;
		AngleLimit(&velAdd);
		if(fabs(velAdd) > 50.0f)
		{
			if(outputVel.direction > lastVelDir)
			{
				outputVel.direction = lastVelDir + 50.0f;
			}
			else if(outputVel.direction < lastVelDir)
			{
				outputVel.direction = lastVelDir - 50.0f;
			}
		}
	}
	
	
	if(outputVel.vel < 1.0f)
	{
		outputVel.direction=tarvel.direction;
	}
	
	if(outputVel.vel>=MAX_TAR_VEL)
	{
		outputVel.vel = MAX_TAR_VEL;
	}
	if(outputVel.vel<=SMALL_VELL)
	{
		outputVel.vel = SMALL_VELL;
	}

	lastVelDir = outputVel.direction;

	tarvel.vel = outputVel.vel;
	tarvel.direction = outputVel.direction;

	GetTarWheel(tarvel.vel, tarvel.direction , tarvel.omega);
}

void controller::VelCtrlTask(void)
{
	vecvel actVel;

	//获取实际车速
	actVel.x = actVelx;
	actVel.y = actVely;

	actVel.GetModule();

	VelControl((Vel_t){actVel.vel,actVel.direction,0.0});
}

void controller::RxCallBack(base_interface::msg::Ops::SharedPtr RxMsg)
{
	pos.point.x = RxMsg->x;
	pos.point.y = RxMsg->y;
	pos.poseAngle = RxMsg->theta;
	actVelx = (pos.point.x-lastpos.point.x)/5*1000;
	actVely = (pos.point.y-lastpos.point.y)/5*1000;
	pos.vel = sqrt(actVelx*actVelx + actVely*actVely);

	g_pos = pos;
	GetVel(actVelx,actVely);
	CaculatePath();

	if (timeFlag_10ms)
	{
		timeFlag_10ms = 0;
		Walk();
		if((walkStatus > WaitForStart) && (walkStatus < STOP))
		{
			VelCtrlTask();
		}
	}
}