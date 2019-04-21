#include "ant_math/ant_math.h"

#define MAX_STEERING_ANGLE 540.0
#define MAX_ROAD_WHEEL_ANGLE 40.0



//方向盘最大转角/前轮最大转角
const float g_steering_gearRatio = MAX_STEERING_ANGLE/MAX_ROAD_WHEEL_ANGLE;

static const float max_side_acceleration = 1.0;


float generate_steeringAngle_by_steeringRadius(float radius)
{
	assert(radius!=0);
	//return asin(AXIS_DISTANCE /radius)*180/M_PI;  //the angle larger
	return atan(AXIS_DISTANCE/radius)*180/M_PI;    //correct algorithm 
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

float generate_max_steering_angle_by_speed(float speed)
{
	float min_steering_radius = speed*speed/max_side_acceleration;
	if(min_steering_radius <3.0)  //radius = 3.0 -> steeringAngle = 30.0
		min_steering_radius = 3.0;
	
	float max_steering_angle = generate_steeringAngle_by_steeringRadius(min_steering_radius);
	if(max_steering_angle > MAX_ROAD_WHEEL_ANGLE - 8.0)
		max_steering_angle = MAX_ROAD_WHEEL_ANGLE -8.0;
	
	return max_steering_angle;
}

int sign(float num)
{
	return num > 0? 1 : -1;
}

float deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

bool load_path_points(std::string file_path,std::vector<gpsMsg_t>& points)
{
	FILE *fp = fopen(file_path.c_str(),"r");
	
	if(fp==NULL)
	{
		ROS_ERROR("open %s failed",file_path.c_str());
		return false;
	}
	
	gpsMsg_t point;
	
	while(!feof(fp))
	{
#if IS_POLAR_COORDINATE_GPS == 1
		fscanf(fp,"%lf\t%lf\t%lf\n",&point.longitude,&point.latitude,&point.yaw);
#else
		fscanf(fp,"%lf\t%lf\t%lf\n",&point.x,&point.y,&point.yaw);
#endif			
		points.push_back(point);
	}
	fclose(fp);
	
	return true;
}






