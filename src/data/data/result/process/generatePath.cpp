#include<iostream>
#include<cmath>
#include<vector>
#include<cstdio>
#include<stdint.h>

using namespace std;

#define _XY 0
#define _LATLON 1

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

enum traffic_sign_t
{
	TrafficSign_None = 0,
	TrafficSign_TrafficLight =1,
	TrafficSign_Avoid = 2,
	TrafficSign_TurnLeft = 3,
	TrafficSign_CarFollow = 4,
	TrafficSign_LaneNarrow = 5,
	TrafficSign_IllegalPedestrian = 6,
	TrafficSign_NoTrafficLight = 7,
	TrafficSign_PickUp = 8,
	TrafficSign_Ambulance = 9,
	TrafficSign_Railway = 10,
	TrafficSign_TempStop = 11,
	TrafficSign_UTurn = 12,
	TrafficSign_School = 13,
	TrafficSign_AvoidStartingCar = 14,
	TrafficSign_OffDutyPerson = 15,
	TrafficSign_Bridge = 16,
	TrafficSign_AccidentArea = 17,
	TrafficSign_JamArea = 18,
	TrafficSign_BusStop = 19,
	TrafficSign_NonVehicle = 20,
	TrafficSign_StopArea = 21, 
	
	TrafficSign_CloseTurnLight = 22,
	TrafficSign_TurnRight = 23,
	TrafficSign_Stop = 24,
};


bool loadPathPoints(std::string file_path,std::vector<gpsMsg_t>& points,bool is_lon_lat)
{
	FILE *fp = fopen(file_path.c_str(),"r");
	
	if(fp==NULL)
	{
		printf("open %s failed\r\n",file_path.c_str());
		return false;
	}
	printf("open %s ok \r\n",file_path.c_str());
	
	gpsMsg_t point;
	
	while(!feof(fp))
	{
		if(is_lon_lat == _LATLON)
			fscanf(fp,"%lf\t%lf\n",&point.longitude,&point.latitude);
		else
		{
			fscanf(fp,"%lf\t%lf\t%lf\t%f\t%f\t%f\t%d\t%d\n",&point.x,&point.y,&point.yaw,&point.curvature,
															&point.maxOffset_left,&point.maxOffset_right,
															&point.traffic_sign,&point.other_info);
			/*printf("%lf\t%lf\t%lf\t%f\t%f\t%f\t%d\t%d\n",point.x,point.y,point.yaw,point.curvature,
															point.maxOffset_left,point.maxOffset_right,
															point.traffic_sign,point.other_info);*/
		}
		points.push_back(point);
	}
	fclose(fp);
	
	return true;
}

bool dumpPathPoints(std::string file_path,std::vector<gpsMsg_t>& points) //xy
{
	FILE *fp = fopen(file_path.c_str(),"w");
	
	if(fp==NULL)
	{
		printf("open %s failed\r\n",file_path.c_str());
		return false;
	}
	printf("open %s ok \r\n",file_path.c_str());
	gpsMsg_t point;
	
	for(size_t i=0;i<points.size();i++)
	{
		point = points[i];
		fprintf(fp,"%.3f\t%.3f\t%.3f\t%.5f\t%.3f\t%.3f\t%d\t%d\r\n",point.x,point.y,point.yaw,point.curvature,
														point.maxOffset_left,point.maxOffset_right,
														point.traffic_sign,point.other_info);
		fflush(fp);
	}
	return true;
}

float dis2Points(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = point1.x - point2.x;
	float y = point1.y - point2.y;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}


float dis2Points_latLon(const gpsMsg_t& point1, const gpsMsg_t& point2,bool is_sqrt)
{
	float x = (point1.longitude -point2.longitude)*111000*cos(point1.latitude);
	float y = (point1.latitude - point2.latitude ) *111000;
	
	if(is_sqrt)
		return sqrt(x*x +y*y);
	return x*x+y*y;
}

size_t findNearestPointIndex(const std::vector<gpsMsg_t>& path_points, const gpsMsg_t& point,bool is_lonlat)
{
	float min_dis = 99999999;
	float dis;
	size_t index = 0;
	for(size_t i=0;i<path_points.size();i++)
	{
		if(is_lonlat == _LATLON)
			dis = dis2Points_latLon(path_points[i],point,true);
		else
			dis = dis2Points(path_points[i],point,true);
		if(dis < min_dis)
		{
			min_dis = dis;
			index = i;
		}
	}
	cout << "mindis:" << min_dis << endl;
	return index;
}

void pathPointsProcess(std::vector<gpsMsg_t>& path_points)
{
	for(size_t i=0; i<path_points.size(); i++)
	{
		;
	}

}

void pointOffset(gpsMsg_t& point,float offset)
{
	if(offset == 0)
		return;
	point.x =  offset * cos(point.yaw) + point.x;
	point.y = -offset * sin(point.yaw) + point.y;
}

size_t findIndexByDis(const vector<gpsMsg_t>& path_points,size_t currentIndex, float t_dis, bool is_up)
{
	float dis;
	if(is_up)
	{
		for(size_t i=currentIndex; i< path_points.size();i++)
		{
			dis = dis2Points(path_points[i],path_points[currentIndex],true);
			if(dis > t_dis)
				return i;
		}	
	}
	else
	{
		for(size_t i=currentIndex; i> 0;i--)
		{
			dis = dis2Points(path_points[i],path_points[currentIndex],true);
			if(dis > t_dis)
				return i;
		}	
	}
	return 0;
}

void markScene(vector<gpsMsg_t>& path_points_xy,size_t startIndex,size_t endIndex,traffic_sign_t scene)
{
	for(size_t i=startIndex; i<endIndex; i++)
	{
		path_points_xy[i].traffic_sign = scene;
	}
}

void setTurnLight(vector<gpsMsg_t>& path_points_xy,size_t startIndex,size_t endIndex, uint8_t status)
{
	for(size_t i=startIndex; i<endIndex; i++)
		path_points_xy[i].other_info = status ;
}


void pathOffset(vector<gpsMsg_t>& path_points_xy,size_t nearest_index,float offset)
{
	size_t A_index = findIndexByDis(path_points_xy,nearest_index,50.0,false);//down
	size_t B_index = findIndexByDis(path_points_xy,nearest_index,20.0,false); //down
	size_t C_index = findIndexByDis(path_points_xy,nearest_index,20.0,true);//up
	size_t D_index = findIndexByDis(path_points_xy,nearest_index,50.0,true); //up
	

	cout << "nearest_index:"<< nearest_index <<"\t "
		 << "A_index:" <<A_index <<"\t"
		  << "B_index:" <<B_index <<"\t"
		   << "C_index:" <<C_index <<"\t"
		    << "D_index:" <<D_index <<"\t";
	
	for(size_t i=A_index; i<B_index; i++)
	{
		float true_offset = offset*(i-A_index)/(B_index-A_index);
		pointOffset(path_points_xy[i],true_offset);  /////offset
		path_points_xy[i].other_info = 2;//right light on
		path_points_xy[i].traffic_sign = TrafficSign_PickUp;
		
	}
	for(size_t i=B_index; i<C_index; i++)
	{
		pointOffset(path_points_xy[i],offset);  /////offset
		path_points_xy[i].other_info = 3;//light down
		
		float percentage = 1.0*(i-B_index)/(C_index-B_index);
		
		if(percentage > 0.3)
			path_points_xy[i].traffic_sign = TrafficSign_Stop;
		else
			path_points_xy[i].traffic_sign = TrafficSign_PickUp;
		
	}
	for(size_t i=C_index; i<D_index; i++)
	{
		float true_offset = offset - offset*(i-C_index)/(D_index-C_index);
		pointOffset(path_points_xy[i],true_offset);  /////offset
		
		float percentage = 1.0*(i-C_index)/(D_index-C_index);
		if(percentage < 0.7)
			path_points_xy[i].other_info = 1;//left light on
		else
			path_points_xy[i].other_info = 3;//light down
	}
}


int main()
{
	vector<gpsMsg_t> path_points_xy;
	
	loadPathPoints("../_end1.txt",path_points_xy,_XY); //x,y
	
	cout << "size:" << path_points_xy.size() <<endl;
	
	setTurnLight(path_points_xy,177,390,1);
	setTurnLight(path_points_xy,390,450,3);
	
	markScene(path_points_xy,490,847,TrafficSign_CarFollow);
	markScene(path_points_xy,976,1060,TrafficSign_IllegalPedestrian);
	setTurnLight(path_points_xy,1100,1250,1);
	markScene(path_points_xy,1100,1150,TrafficSign_TurnLeft);
	markScene(path_points_xy,1150,1250,TrafficSign_NoTrafficLight);
	setTurnLight(path_points_xy,1250,1300,3);
	
	pathOffset(path_points_xy,1336,2.8); //pickup
	markScene(path_points,1400,1570,TrafficSign_Ambulance);
	markScene
	
	
	
	
	markScene(path_points_xy,)
	
	
	pathOffset(path_points_xy,1620,2.5);  // points index offset
	
	//pathOffset(path_points_xy,1000,10.0);
	
	//markScene(path_points_xy,100,200,TrafficSign_Ambulance);
	
	
	dumpPathPoints("../final.txt",path_points_xy);
	
	return 0;
	
}


