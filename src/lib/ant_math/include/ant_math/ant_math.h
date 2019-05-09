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
#include<exception>

#define IS_POLAR_COORDINATE_GPS 0

#define WHEEL_DISTANCE  1.2
#define AXIS_DISTANCE  1.5
#define MAX_SPEED 30.0

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	
	double x;
	double y;
	
	float curvature;
	
}gpsMsg_t;

extern const float g_steering_gearRatio;
extern const float g_vehicle_width;
extern const float g_vehicle_length;
extern const float g_max_deceleration;


inline float generateRoadwheelAngleByRadius(const float& radius)
{
	assert(radius!=0);
	//return asin(AXIS_DISTANCE /radius)*180/M_PI;  //the angle larger
	return atan(AXIS_DISTANCE/radius)*180/M_PI;    //correct algorithm 
}

inline double sinDeg(const double& deg)
{
	return sin(deg*M_PI/180.0);
}

inline float saturationEqual(float value,float limit)
{
	//ROS_INFO("value:%f\t limit:%f",value,limit);
	assert(limit>0);
	if(value>limit)
		value = limit;
	else if(value < -limit)
		value = -limit;
	return value;
}

inline int sign(float num)
{
	return num > 0? 1 : -1;
}

inline float deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

inline float generateDangerDistanceBySpeed(const float &speed)
{
	return 0.5* speed * speed /g_max_deceleration  + 3.0; 
}

inline float generateSafetyDisByDangerDis(const float &danger_dis)
{
	return danger_dis *3 + 5.0;
}

float limitRoadwheelAngleBySpeed(const float& angle, const float& speed);
float limitSpeedByPathCurvature(const float& speed,const float& curvature);
float limitSpeedByCurrentRoadwheelAngle(float speed,float angle);
bool   loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points);
float  calculateDis2path(const double& X_,const double& Y_,
						 const std::vector<gpsMsg_t>& path_points, 
						 const size_t& target_point_index,
						 size_t * const nearest_point_index_ptr=NULL);
						 
float maxRoadWheelAngleWhenChangeLane(const float& offset,const float& distance);
float generateDangerDistanceBySpeed(const float &speed);
float generateMaxTolarateSpeedByCurvature(const float& curvature);
float generateMaxTolarateSpeedByCurvature(const std::vector<gpsMsg_t>& path_points,
											const size_t& nearest_point_index,
											const size_t& target_point_index);






#endif


