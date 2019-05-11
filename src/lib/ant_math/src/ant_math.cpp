#include "ant_math/ant_math.h"

#define MAX_STEERING_ANGLE 540.0
#define MAX_ROAD_WHEEL_ANGLE 25.0



//方向盘最大转角/前轮最大转角
const float g_steering_gearRatio = MAX_STEERING_ANGLE/MAX_ROAD_WHEEL_ANGLE;

const float g_vehicle_width = 1.7 ;// m
const float g_vehicle_length = 3.5; 

const float g_max_deceleration = 5.0; // m/s/s

static const float max_side_acceleration = 1.9; // m/s/s



float limitRoadwheelAngleBySpeed(const float& angle, const float& speed)
{
	float min_steering_radius = speed*speed/max_side_acceleration;
	if(min_steering_radius <3.0)  //radius = 3.0 -> steeringAngle = 30.0
		min_steering_radius = 3.0;
	
	float max_roadwheelAngle = fabs(generateRoadwheelAngleByRadius(min_steering_radius));
	if(max_roadwheelAngle > MAX_ROAD_WHEEL_ANGLE - 5.0)
	   max_roadwheelAngle = MAX_ROAD_WHEEL_ANGLE -5.0;
	//ROS_INFO("max_angle:%f\t angle:%f",max_roadwheelAngle,angle);
	return saturationEqual(angle,max_roadwheelAngle);
}

float limitSpeedByCurrentRoadwheelAngle(float speed,float angle)
{
	float steering_radius = fabs(AXIS_DISTANCE/tan(angle*M_PI/180.0));
	float max_speed =  sqrt(steering_radius*max_side_acceleration);
	return speed>max_speed? max_speed: speed;
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
		fscanf(fp,"%lf\t%lf\t%lf\t%f\t%f\t%f\t%d\t%d\n",&point.x,&point.y,&point.yaw,&point.curvature,
														&point.maxOffset_left,&point.maxOffset_right,
														&point.traffic_sign,&point.other_info);
#endif
		points.push_back(point);
	}
	fclose(fp);
	
	return true;
}

float calculateDis2path(const double& X_,const double& Y_,
						 const std::vector<gpsMsg_t>& path_points, 
						 const size_t& target_point_index,
						 size_t * const nearest_point_index_ptr)
{
	if(target_point_index == 0)
	{
		if(nearest_point_index_ptr != NULL)
			*nearest_point_index_ptr = 0;
	
		//the direction of side c 
		//float yaw_of_c = (path_points[first_point_index].yaw + path_points[second_point_index].yaw)/2;
		float yaw_of_c = atan2(path_points[1].x-path_points[0].x,
										   path_points[1].y-path_points[0].y);
				
		//object : world coordination to local coordination
		float x = (X_-path_points[0].x) * cos(yaw_of_c) - (Y_-path_points[0].y) * sin(yaw_of_c);
		//float y = (X_-path_points[first_point_index].x) * sin(yaw_of_c) + (Y_-path_points[first_point_index].y) * cos(yaw_of_c);
	
		return x;
	}
	
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
	
	int is_yawReverse = 0;
	
	if(dis2last_target <dis2target && dis2next_target > dis2target) //downward
	{
		is_yawReverse = 1;
		for(size_t i=1;true;i++)
		{	
		/*   prevent size_t index 0-1 data overflow    */
			if(target_point_index >= i)
				second_point_index = target_point_index-i;
			else
			{
				first_point_index = 0;
				second_point_index = 1;
				break;
			}
		/*   prevent size_t index 0-1 data overflow    */
			
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
			if(second_point_index >= path_points.size())
			{
				throw "point index out of range";
				return 0;
			}
			
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
		//first_point_index = target_point_index-1;
		first_point_index = target_point_index;
		
		second_point_index = target_point_index +1;
	}
		
	if(nearest_point_index_ptr != NULL)
		*nearest_point_index_ptr = (first_point_index+second_point_index)/2;
	
	//the direction of side c 
	//float yaw_of_c = (path_points[first_point_index].yaw + path_points[second_point_index].yaw)/2;
	float yaw_of_c = is_yawReverse*M_PI + atan2(path_points[second_point_index].x-path_points[first_point_index].x,
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
	
	//float max_speed =  sqrt(1.0/fabs(curvature)*max_side_acceleration) *3.6;
	float max_speed =  sqrt(1.0/fabs(curvature)*1.5) *3.6;
	return speed>max_speed? max_speed: speed;
}

//km/h
float generateMaxTolarateSpeedByCurvature(const float& curvature)
{
	if(curvature==0.0)
		return 100.0;

	return sqrt(1.0/fabs(curvature)*1.5) *3.6;
}

float generateMaxTolarateSpeedByCurvature(const std::vector<gpsMsg_t>& path_points,
											const size_t& nearest_point_index,
											const size_t& target_point_index)
{
	float max_cuvature = 0.0001;
	size_t endIndex = target_point_index + 10;
	if(endIndex >= path_points.size())
		endIndex = path_points.size() -1;
		
	for(size_t i=nearest_point_index; i < endIndex; i++)
	{
		if(fabs(path_points[i].curvature) > max_cuvature)
			max_cuvature = fabs(path_points[i].curvature);
	}
	return sqrt(1.0/max_cuvature*1.5) *3.6;
}

float limitSpeedByLateralAndYawErr(float speed,float latErr,float yawErr)
{
	///??
}

//offset: change lane offset
//distance: longitudianal displacement of vehicle in the course of change lane
float maxRoadWheelAngleWhenChangeLane(const float& offset,const float& distance)
{
	float theta = 2*atan(fabs(offset)/distance);
	float radius = 0.5*distance/sin(theta);
	return generateRoadwheelAngleByRadius(radius);
}



/*
void generateMaxOffset(const std::vector<gpsMsg_t>& path_points, 
					   size_t nearest_point_index,
					   size_t target_point_index,
					   float& max_left, float& max_right)
{
	float left = -10; // a small number
	float right = 10; //a big number
	
	size_t endIndex = target_point_index + 10;
	
	
}

//given the startIndex and expect distance  find the point index
size_t indexForGivenDis(const std::vector<gpsMsg_t>& path_points, size_t startIndex,float dis)
{
	float sum_dis = 0.0;
	while(ros::ok())
	{
		sum_dis	+= disBetweenPoints(path_points[startIndex],path_points[startIndex+5]);
		
		startIndex += 5;
		
		if(sum_dis > dis)
			return startIndex;
	}
}

float disBetweenPoints(const gpsMsg_t& point1, const gpsMsg_t& point2)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	return sqrt(x*x+y*y);
}

*/

