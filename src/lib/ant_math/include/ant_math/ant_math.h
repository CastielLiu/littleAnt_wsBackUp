#ifndef ANT_MATH_H_
#define ANT_MATH_H_

#include<cstring>
#include<cmath>
#include<assert.h>
#include<string>
#include<vector>
#include<cstdio>
#include<ros/ros.h>
#include<limits.h>

#define IS_POLAR_COORDINATE_GPS 0

#define WHEEL_DISTANCE  1.2
#define AXIS_DISTANCE  1.5

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	
	double x;
	double y;
	
}gpsMsg_t;


float generate_steeringAngle_by_steeringRadius(float radius);
double sin_deg(double deg); 
float limit_steeringAngle(float angle,float limit);
int sign(float num);
float deg2rad(float deg);
float generate_max_steering_angle_by_speed(float speed);
bool load_path_points(std::string file_path,std::vector<gpsMsg_t>& points);

extern const float g_steering_gearRatio;
extern const float g_vehicle_width;




#endif


