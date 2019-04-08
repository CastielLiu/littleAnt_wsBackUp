#include "ant_math/ant_math.h"
#include<cstring>
#include<cmath>
#include<assert.h>

float generate_steeringAngle_by_steeringRadius(float radius)
{
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
