#ifndef ANT_MATH_H_
#define ANT_MATH_H_

#define WHEEL_DISTANCE  1.2
#define AXIS_DISTANCE  1.5


float generate_steeringAngle_by_steeringRadius(float radius);
double sin_deg(double deg); 
float limit_steeringAngle(float angle,float limit);
int sign(float num);
float deg2rad(float deg);
float generate_max_steering_angle_by_speed(float speed);

extern const float g_steering_gearRatio;

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	
	double x;
	double y;
	
}gpsMsg_t;


#endif


