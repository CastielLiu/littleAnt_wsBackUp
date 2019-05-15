#include<iostream>
#include<vector>

using namespace std;

typedef struct
{
	double longitude;
	double latitude;
	double yaw;
	
	double x;
	double y;
	
	float curvature;
	
	float maxOffset_left;
	float maxOffset_right;
	uint8_t traffic_sign;
	uint8_t other_info;
	
}gpsMsg_t;


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

int main()
{
	vector<gpsMsg_t> path_points;
	loadPathPoints("")
	
}
