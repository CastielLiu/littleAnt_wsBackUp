#include "ant_math/ant_math.h"

#define MAX_STEERING_ANGLE 540.0
#define MAX_ROAD_WHEEL_ANGLE 25.0



//方向盘最大转角/前轮最大转角
const float g_steering_gearRatio = MAX_STEERING_ANGLE/MAX_ROAD_WHEEL_ANGLE;

const float g_vehicle_width = 1.8 ;// m
const float g_vehicle_length = 3.5; 

static const float max_side_acceleration = 1.2; // m/s/s


float generateRoadwheelAngleByRadius(float radius)
{
	assert(radius!=0);
	//return asin(AXIS_DISTANCE /radius)*180/M_PI;  //the angle larger
	return atan(AXIS_DISTANCE/radius)*180/M_PI;    //correct algorithm 
}

double sinDeg(const double& deg)
{
	return sin(deg*M_PI/180.0);
}

float saturationEqual(float value,float limit)
{
	//ROS_INFO("value:%f\t limit:%f",value,limit);
	assert(limit>0);
	if(value>limit)
		value = limit;
	else if(value < -limit)
		value = -limit;
	return value;
}


float limitRoadwheelAngleBySpeed(const float& angle, const float& speed)
{
	float min_steering_radius = speed*speed/max_side_acceleration;
	if(min_steering_radius <3.0)  //radius = 3.0 -> steeringAngle = 30.0
		min_steering_radius = 3.0;
	
	float max_roadwheelAngle = fabs(generateRoadwheelAngleByRadius(min_steering_radius));
	if(max_roadwheelAngle > MAX_ROAD_WHEEL_ANGLE - 5.0)
	   max_roadwheelAngle = MAX_ROAD_WHEEL_ANGLE -5.0;
	return saturationEqual(angle,max_roadwheelAngle);
}

float limitSpeedByCurrentRoadwheelAngle(float speed,float angle)
{
	float steering_radius = fabs(AXIS_DISTANCE/tan(angle*M_PI/180.0));
	float max_speed =  sqrt(steering_radius*max_side_acceleration);
	return speed>max_speed? max_speed: speed;
}

int sign(float num)
{
	return num > 0? 1 : -1;
}

float deg2rad(float deg)
{
	return  (deg/180.0)*M_PI;
}

bool loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points)
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
		fscanf(fp,"%lf\t%lf\t%lf\t%f\n",&point.x,&point.y,&point.yaw,&point.curvature);
#endif			
		points.push_back(point);
	}
	fclose(fp);
	
	return true;
}


float calculateDis2path(const double& X_,const double& Y_,
						 const std::vector<gpsMsg_t>& path_points, 
						 const size_t& target_point_index)
{
	//ROS_INFO("path_points.size:%d\t target_point_index:%d",path_points.size(),target_point_index);
	
	//this target is tracking target,
	//let the target points as the starting point of index
	//Judging whether to index downward or upward
	float dis2target = pow(path_points[target_point_index].x - X_, 2) + 
					   pow(path_points[target_point_index].y - Y_, 2) ;
	
	float dis2next_target = pow(path_points[target_point_index+1].x - X_, 2) + 
							pow(path_points[target_point_index+1].y - Y_, 2) ;
							
	float dis2last_target = pow(path_points[target_point_index-1].x - X_, 2) + 
					        pow(path_points[target_point_index-1].y - Y_, 2) ;
	
	//std::cout << sqrt(dis2target)<<"\t"<< sqrt(dis2next_target) <<"\t"<< sqrt(dis2last_target) << std::endl;
	
	float first_dis ,second_dis ;  //a^2 b^2 
	size_t first_point_index,second_point_index;
	
	first_dis = dis2target;
	first_point_index = target_point_index;
	
	int direction = 1;
	
	if(dis2last_target <dis2target && dis2next_target > dis2target) //downward
	{
		direction = -1;
		for(size_t i=1;true;i++)
		{
			second_point_index = target_point_index-i;
			
			second_dis = pow(path_points[second_point_index].x - X_, 2) + 
						 pow(path_points[second_point_index].y - Y_, 2) ;
			
			if(second_dis < first_dis) //continue 
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else if(dis2next_target < dis2target && dis2last_target > dis2target) //upward
	{
		for(size_t i=1;true;i++)
		{
			second_point_index = target_point_index + i;
			second_dis = pow(path_points[second_point_index].x - X_, 2) + 
						 pow(path_points[second_point_index].y - Y_, 2) ;

			if(second_dis < first_dis) //continue
			{
				first_dis = second_dis;
				first_point_index = second_point_index;
			}
			else  //end
				break;
		}
	}
	else //midile
	{
		first_point_index = target_point_index-1;
		second_point_index = target_point_index +1;
	}
	
	//the direction of side c 
	//float yaw_of_c = (path_points[first_point_index].yaw + path_points[second_point_index].yaw)/2;
	float yaw_of_c = direction * atan2(path_points[second_point_index].x-path_points[first_point_index].x,
									   path_points[second_point_index].y-path_points[first_point_index].y);
						   
	//object : world coordination to local coordination
	float x = (X_-path_points[first_point_index].x) * cos(yaw_of_c) - (Y_-path_points[first_point_index].y) * sin(yaw_of_c);
	//float y = (X_-path_points[first_point_index].x) * sin(yaw_of_c) + (Y_-path_points[first_point_index].y) * cos(yaw_of_c);
	
	//ROS_ERROR("index1:%d\t index2:%d",first_point_index,second_point_index);
	return x;
}

float limitSpeedByPathCurvature(const float& speed,const float& curvature)
{
	if(curvature == 0.0)
		return speed;
	
	float max_speed =  sqrt(1.0/curvature*max_side_acceleration) *3.6;
	return speed>max_speed? max_speed: speed;
}

float limitSpeedByLateralAndYawErr(float speed,float latErr,float yawErr)
{
	///??
}








