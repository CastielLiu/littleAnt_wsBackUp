#ifndef ANT_MATH_H_
#define ANT_MATH_H_

#define WHEEL_DISTANCE  1.2
#define AXIS_DISTANCE  1.5


float generate_steeringAngle_by_steeringRadius(float radius);
double sin_deg(double deg); 
float limit_steeringAngle(float angle,float limit);
int sign(float num);

extern const float g_steering_gearRatio;




#endif


