#include "ant_math/ant_math.h"
#include<cstring>
#include<cmath>
#include<assert.h>

float generate_steeringAngle_by_steeringRadius(float radius)
{
	assert(radius!=0);
	return asin(AXIS_DISTANCE /radius)*180/M_PI;
}

double sin_deg(double deg)
{
	return sin(deg*M_PI/180.0);
}

float limit_steeringAngle(float angle,float limit)
{
	assert(limit>0);
	if(angle>limit)
		angle = limit;
	else if(angle < -limit)
		angle = -limit;
	return angle;
}

int sign(float num)
{
	return num > 0? 1 : -1;
}

float deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

//方向盘最大转角/前轮最大转角
const float g_steering_gearRatio = 540.0/40.0;
