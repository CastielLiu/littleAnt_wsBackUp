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


float  generateRoadwheelAngleByRadius(const float& radius);
double sinDeg(const double& deg); 
float  saturationEqual(float value,float limit);
int    sign(float num);
float  deg2rad(float deg);
float limitRoadwheelAngleBySpeed(const float& angle, const float& speed);
float limitSpeedByPathCurvature(const float& speed,const float& curvature);
float limitSpeedByCurrentRoadwheelAngle(float speed,float angle);
bool   loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points);
float  calculateDis2path(const double& X_,const double& Y_,
						 const std::vector<gpsMsg_t>& path_points, 
						 const size_t& target_point_index);
						 
float maxRoadWheelAngleWhenChangeLane(const float& offset,const float& distance);
float generateDangerDistanceBySpeed(const float &speed);
float generateSafetyDisByDangerDis(const float &danger_dis);
float generateMaxTolarateSpeedByCurvature(const float& curvature);


extern const float g_steering_gearRatio;
extern const float g_vehicle_width;
extern const float g_vehicle_length;




#endif


