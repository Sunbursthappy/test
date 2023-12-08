#include <control/path.hpp>
#include <control/pathfollowing.hpp>
#include <control/Bspline.hpp>

Pose_t circle[CIRCLE_PATH_NUM_DEF] =
{
{   0.000f ,   0.000f ,0.0f ,TARGET_VEL/PERCENT_PATH},
{   0.000f , 168.100f ,0.0f ,MAX_TAR_VEL},
{   0.000f , 336.201f ,0.0f ,MAX_TAR_VEL},
{   0.000f , 504.301f ,0.0f ,MAX_TAR_VEL},
{   0.000f , 672.402f ,0.0f ,MAX_TAR_VEL},
{   0.000f , 840.502f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,1008.603f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,1176.703f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,1344.804f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,1512.904f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,1681.004f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,1849.105f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,2017.205f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,2185.306f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,2353.406f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,2521.507f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,2689.607f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,2857.707f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,3025.808f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,3193.908f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,3362.009f ,0.0f ,MAX_TAR_VEL},
{   0.000f ,3530.109f ,0.0f ,MAX_TAR_VEL},
{  12.983f ,3701.514f ,0.0f ,MAX_TAR_VEL},
{  51.594f ,3857.959f ,0.0f ,MAX_TAR_VEL},
{ 114.832f ,4006.171f ,0.0f ,MAX_TAR_VEL},
{ 201.054f ,4142.302f ,0.0f ,MAX_TAR_VEL},
{ 273.840f ,4241.038f ,0.0f ,MAX_TAR_VEL},
{ 347.965f ,4338.774f ,0.0f ,MAX_TAR_VEL},
{ 422.990f ,4435.822f ,0.0f ,MAX_TAR_VEL},
{ 498.482f ,4532.507f ,0.0f ,MAX_TAR_VEL},
{ 574.011f ,4629.163f ,0.0f ,MAX_TAR_VEL},
{ 649.155f ,4726.119f ,0.0f ,MAX_TAR_VEL},
{ 723.499f ,4823.689f ,0.0f ,MAX_TAR_VEL},
{ 797.003f ,4921.894f ,0.0f ,MAX_TAR_VEL},
{ 870.351f ,5020.215f ,0.0f ,MAX_TAR_VEL},
{ 944.306f ,5118.081f ,0.0f ,MAX_TAR_VEL},
{1019.621f ,5214.902f ,0.0f ,MAX_TAR_VEL},
{1097.048f ,5310.039f ,0.0f ,MAX_TAR_VEL},
{1177.330f ,5402.777f ,0.0f ,MAX_TAR_VEL},
{1261.187f ,5492.291f ,0.0f ,MAX_TAR_VEL},
{1349.280f ,5577.633f ,0.0f ,MAX_TAR_VEL},
{1442.165f ,5657.723f ,0.0f ,MAX_TAR_VEL},
{1540.216f ,5731.386f ,0.0f ,MAX_TAR_VEL},
{1643.305f ,5797.821f ,0.0f ,MAX_TAR_VEL},
{1750.840f ,5856.787f ,0.0f ,MAX_TAR_VEL},
{1862.202f ,5908.166f ,0.0f ,MAX_TAR_VEL},
{1976.776f ,5951.917f ,0.0f ,MAX_TAR_VEL},
{2093.971f ,5988.073f ,0.0f ,MAX_TAR_VEL},
{2213.224f ,6016.725f ,0.0f ,MAX_TAR_VEL},
{2334.011f ,6038.011f ,0.0f ,MAX_TAR_VEL},
{2455.816f ,6052.402f ,0.0f ,MAX_TAR_VEL},
{2578.105f ,6061.979f ,0.0f ,MAX_TAR_VEL},
{2700.557f ,6069.211f ,0.0f ,MAX_TAR_VEL},
{2823.007f ,6076.484f ,0.0f ,MAX_TAR_VEL},
{2945.275f ,6086.293f ,0.0f ,MAX_TAR_VEL},
{3066.983f ,6101.465f ,0.0f ,MAX_TAR_VEL},
{3187.254f ,6125.386f ,0.0f ,MAX_TAR_VEL},
{3304.222f ,6162.063f ,0.0f ,MAX_TAR_VEL},
{3414.417f ,6215.618f ,0.0f ,MAX_TAR_VEL},
{3512.770f ,6288.587f ,0.0f ,MAX_TAR_VEL},
{3595.881f ,6378.603f ,0.0f ,MAX_TAR_VEL},
{3664.147f ,6480.398f ,0.0f ,MAX_TAR_VEL},
{3718.847f ,6590.114f ,0.0f ,MAX_TAR_VEL},
{3761.163f ,6705.193f ,0.0f ,MAX_TAR_VEL},
{3791.896f ,6823.899f ,0.0f ,MAX_TAR_VEL},
{3811.392f ,6944.962f ,0.0f ,MAX_TAR_VEL},
{3819.556f ,7067.311f ,0.0f ,MAX_TAR_VEL},
{3816.368f ,7189.893f ,0.0f ,MAX_TAR_VEL},
{3802.116f ,7311.689f ,0.0f ,MAX_TAR_VEL},
{3777.202f ,7431.759f ,0.0f ,MAX_TAR_VEL},
{3742.095f ,7549.255f ,0.0f ,MAX_TAR_VEL},
{3697.278f ,7663.404f ,0.0f ,MAX_TAR_VEL},
{3643.231f ,7773.484f ,0.0f ,MAX_TAR_VEL},
{3580.400f ,7878.798f ,0.0f ,MAX_TAR_VEL},
{3509.200f ,7978.644f ,0.0f ,MAX_TAR_VEL},
{3430.016f ,8072.282f ,0.0f ,MAX_TAR_VEL},
{3343.227f ,8158.918f ,0.0f ,MAX_TAR_VEL},
{3249.694f ,8238.236f ,0.0f ,MAX_TAR_VEL},
{3150.930f ,8310.955f ,0.0f ,MAX_TAR_VEL},
{3048.346f ,8378.191f ,0.0f ,MAX_TAR_VEL},
{2943.139f ,8441.256f ,0.0f ,MAX_TAR_VEL},
{2836.311f ,8501.542f ,0.0f ,MAX_TAR_VEL},
{2728.726f ,8560.468f ,0.0f ,MAX_TAR_VEL},
{2621.172f ,8619.453f ,0.0f ,MAX_TAR_VEL},
{2514.443f ,8679.912f ,0.0f ,MAX_TAR_VEL},
{2409.160f ,8742.857f ,0.0f ,MAX_TAR_VEL},
{2305.154f ,8807.891f ,0.0f ,MAX_TAR_VEL},
{2202.030f ,8874.317f ,0.0f ,MAX_TAR_VEL},
{2099.362f ,8941.446f ,0.0f ,MAX_TAR_VEL},
{1996.705f ,9008.592f ,0.0f ,MAX_TAR_VEL},
{1893.610f ,9075.062f ,0.0f ,MAX_TAR_VEL},
{1789.639f ,9140.152f ,0.0f ,MAX_TAR_VEL},
{1684.381f ,9203.139f ,0.0f ,MAX_TAR_VEL},
{1577.482f ,9263.290f ,0.0f ,MAX_TAR_VEL},
{1468.872f ,9320.300f ,0.0f ,MAX_TAR_VEL},
{1358.765f ,9374.360f ,0.0f ,MAX_TAR_VEL},
{1247.363f ,9425.704f ,0.0f ,MAX_TAR_VEL},
{1134.850f ,9474.566f ,0.0f ,MAX_TAR_VEL},
{1021.390f ,9521.186f ,0.0f ,MAX_TAR_VEL},
{ 907.128f ,9565.804f ,0.0f ,MAX_TAR_VEL},
{ 792.194f ,9608.662f ,0.0f ,MAX_TAR_VEL},
{ 676.703f ,9650.000f ,0.0f ,0},
};

Pose_t g_pos;

vecvel::vecvel()
{
  x         = 0;
  y         = 0;
  vel       = 0;
  direction = 0;
}

void vecvel::GetModule()
{
  vel = sqrt(x*x + y*y);
  direction = atan2(y,x)/PI*180;
}

void vecvel::TransModule()
{
  x = vel*cos(direction/180*PI);
  y = vel*sin(direction/180*PI);
}

vecvel vecvel::operator+(const vecvel &t) const
{
  vecvel sum;
  sum.x = x + t.x;
  sum.y = y + t.y;
  sum.GetModule();
  return sum;
}
vecvel vecvel::operator-(const vecvel &t) const
{
  vecvel dif;
  dif.x = x - t.x;
  dif.y = y - t.y;
  dif.GetModule();
  return dif;
}

Point_t Getpps(Pose_t pos)
{
  Point_t pps;
  pps.x = pos.point.x;
  pps.y = pos.point.y;
  return pps;
}

/***********************************************************************************
* @name 		CaculatePath
* @brief  		计算走过的里程
* @param  	
* @retval 	
**********************************************************************************/
float lengthWheelTraveled = 0.0f;
//0不记录
//1记录距离
//默认初始就开始记录距离
_Bool CaculateLenFlag = 0;
float posppsXOld = 0.0f,posppsYOld = 0.0f;
float errDirection = 0.0f;
extern FILE *fpWrite;
Point_t calculatePos;
static uint8_t clealLenFlag = 0;

//通过定位系统计算机器人行走的路径长度
float Velx,Vely;
void GetVel(float actVelx,float actVely)
{
	Velx = actVelx;
	Vely = actVely;
}
void CaculatePath(void)
{
	float robotVel = 0;
	float err = -0.02f;
	float reducePath = 0.0f;
	PointU_t virtualPosition = {0.0f};
	if(CaculateLenFlag == 1)
	{
		//第一次进这个函数,数组数据清零
		err = Get_AverLength(g_pos.point.x, g_pos.point.y);
		clealLenFlag = 1;
		robotVel = sqrt(Velx*Velx + Vely*Vely);
		if(err> 0.2f && robotVel > 4.2f)
		{
			virtualPosition = SerchVirtualPoint2(GetPath());
			err = err * cosf((errDirection - virtualPosition.direction)/57.3f);
			if(err>=0.0f)
			{
				if(err<300.0f)
				{
					lengthWheelTraveled += err;
				}
				else
				{
					lengthWheelTraveled += err;
				}
			}
			if(lengthWheelTraveled <= 0.0f)
			{
				lengthWheelTraveled = 0.0f;
			} 
		}
	}
	else
	{
		clealLenFlag = 0;
	}
}

//运行路径长度记录
void UpdateLenBegin(void)
{
	CaculateLenFlag = 1;
}

//停止路径长度记录
void UpdateLenStop(void)
{
	CaculateLenFlag = 0;
}

//增加距离
void AddPath(float dis)
{
	lengthWheelTraveled += dis;
}

//减小距离
void ReducePath(float dis)
{
	lengthWheelTraveled -= (dis);
}

/***********************************************************************************
* @name 		GetPath
* @brief  	返回里程数
* @param  	无
* @retval 	lengthWheelTraveled
**********************************************************************************/
float GetPath(void)
{
	return lengthWheelTraveled;
}

void ClearPathLen(void)
{
	posppsXOld = g_pos.point.x;
	posppsYOld = g_pos.point.y;
	lengthWheelTraveled = 0.0f;
	UpdateLenStop();
}


float g_err;
float Get_AverLength(float ppsx, float ppsy)
{
	const uint16_t number = 19;
	static int   numberCnt = 0;
	static float ppsX[19] = {0};
	static float ppsY[19] = {0};
	static float calculateY = 0.f;
	static float ppsCos  = 0.f;
	static float ppsa2 = 0.f;
	static float ppsb2 = 0.f;
	static float ppsc2  = 0.f;
	static float ppsOldX = 0.f;//用定位系统记录长度
	static float ppsOldY = 0.f;
	float  errLen = 0.f;

	if (clealLenFlag == 0)
	{
		numberCnt = 0;
		for(int i = 0;i < (number);i++)
		{
			ppsY[i] = 0.f;
			ppsX[i] = 0.f;
			calculatePos.x = 0.f;
			calculatePos.y = 0.f;
			posppsXOld = 0.f;
			posppsYOld = 0.f;
		}

	}
	ppsX[18] = ppsx;
	ppsY[18] = ppsy;

	if(numberCnt < number)
	{
		ppsX[numberCnt] = ppsx;
		ppsY[numberCnt] = ppsy;
	}

	if(numberCnt == number -1)
	{
		ppsa2  = pow((ppsX[0]-ppsX[9]),2)  + pow((ppsY[0]-ppsY[9]),2);
		ppsb2  = pow((ppsX[9]-ppsX[18]),2) + pow((ppsY[9]-ppsY[18]),2);
		ppsc2  = pow((ppsX[0]-ppsX[18]),2) + pow((ppsY[0]-ppsY[18]),2);
		ppsCos = (ppsa2+ppsb2-ppsc2)/(2.0 * sqrtf(ppsa2) * sqrtf(ppsb2));
	
		
		if ((2.0 * sqrtf(ppsa2) * sqrtf(ppsb2)) == 0.f)
		{
			ppsCos = 0.f;
    }
		calculatePos.x = ppsX[18];
		calculatePos.y = ppsY[18];
		//防止第一次值特别小
		if (posppsXOld < 10.f)
		{
			posppsXOld = calculatePos.x;
			posppsYOld = calculatePos.y;

		}
		errLen = sqrt((calculatePos.x - posppsXOld)*(calculatePos.x - posppsXOld) + (calculatePos.y - posppsYOld)*(calculatePos.y - posppsYOld));
		errDirection = 57.3f*atan2f((calculatePos.y - posppsYOld) , (calculatePos.x- posppsXOld));

		posppsXOld = ppsX[18];
		posppsYOld = ppsY[18];
	
		for(int i = 0;i < (number -1);i++)
		{
			ppsY[i] = ppsY[i+1];
			ppsX[i] = ppsX[i+1];
		}
	}
	else
	{
		errLen = sqrt((ppsOldX - ppsx)*(ppsOldX - ppsx) + (ppsOldY - ppsy)*(ppsOldY - ppsy));
		if (errLen > 20.f)
		{
			errLen = 20.f;
		}
		errDirection = 57.3f*atan2f((ppsy - ppsOldY) , (ppsx- ppsOldX));
	}
	ppsOldX = ppsx;
	ppsOldY = ppsy;
	numberCnt++;
	if (numberCnt > number -1)
	{
		numberCnt = number -1;
	}
	g_err = errLen;
	return errLen;
}

Vel_t CalcWheelSpeed(float vel, float direction, float omega, int wheelNum, float postureAngle)
{
	Vel_t sumVel = {0.0f};
	float velX, velY = 0.0f;
	float velN, velNDirection = 0.0f;
	float sumVelX, sumVelY = 0.0f;
	//计算平移速度的X，Y分量
	velX = vel * cosf(PI/180*(direction));
	velY = vel * sinf(PI/180*(direction));
	
	//计算旋转的线速度
	switch (wheelNum)
	{
		case 1:
			velN = PI/180*(omega) * MOVEBASE_RADIUS;
			velNDirection = ONE_VERTICAL_ANG + postureAngle;
			AngleLimit(&velNDirection);
			break;
		case 2:
			velN = PI/180*(omega) * MOVEBASE_RADIUS;
			velNDirection = TWO_VERTICAL_ANG + postureAngle;
			AngleLimit(&velNDirection);
			break;
		case 3:
			velN = PI/180*(omega) * MOVEBASE_RADIUS;
			velNDirection = THREE_VERTICAL_ANG + postureAngle;
			AngleLimit(&velNDirection);
			break;
		default:
			break;
	}
	

	//防止速度过快，姿态角转不过来
	if (fabs(omega) > 150.0f)
	{
		velX = velX * 0.80;
		velY = velY * 0.80;
	}

	//计算和速度大小和方向
	sumVelX = velX + velN * cosf(PI/180*(velNDirection));
	sumVelY = velY + velN * sinf(PI/180*(velNDirection));

	sumVel.vel = sqrtf(sumVelX * sumVelX + sumVelY * sumVelY);

	//计算合成速度方向时将0向量单独处理
	if (sumVel.vel > 0.01f)
	{
		sumVel.direction = 180/PI*(atan2f(sumVelY, sumVelX));
	}
	else
	{
		sumVel.direction = direction;
	}

	return sumVel;
}