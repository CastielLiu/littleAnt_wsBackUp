#include<iostream>
#include"serial.h"
#include"little_ant_msgs/ControlCmd1.h"
#include"little_ant_msgs/ControlCmd2.h"
#include<boost/thread.hpp>
#include<boost/bind.hpp>

#include<ros/ros.h>


using namespace std;
uint8_t generate_check_sum(uint8_t *buf,int len);
void getMsg();

const int BufLen = 120;
uint8_t recvBuf[BufLen];
uint8_t *msgHeader = NULL;
uint8_t *headPtr = recvBuf;
uint8_t *rearPtr = recvBuf;

uint8_t msgBuf[12];
boost::mutex mutex_;

std::string port_name_;

int main(int argc ,char **argv)
{
	ros::init(argc,argv,"key_control");
	
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("port_name",port_name_,"/dev/ttyUSB0");
	
	ros::Publisher pub1 = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",10);
	ros::Publisher pub2 = nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",10);
	
	boost::thread parseThread(boost::bind(&getMsg));//thread
	usleep(100000);
	
	little_ant_msgs::ControlCmd1 cmd1;
	little_ant_msgs::ControlCmd2 cmd2;
	
	float speed = 0, steer = 0, steer_right = 0, brake = 0;
	bool didi = 0, left_light = 0, right_light = 0, D_change = 0, hand_brake = 0,mode=0 , emergencyBrake=0;

	while(ros::ok())
	{
	///////////////////////////////////////////////////////////////////////////////		
		/*for(int i=0;i<12;i++)
    	{
    		printf("%x\t",recvBuf[i]);
    	}
    	cout <<endl;*/
    	
		//cout <<"running..." <<endl;
		if ((msgBuf[5] | 0xEF) == 0xef)
		{
			if (mode == 0)
				mode = 1;
			else
				mode = 0;
			while((msgBuf[5] | 0xEF) == 0xef)
				usleep(1000);
		}

		if((msgBuf[5]|0x7F)==0x7f)
		{
			if (left_light == 0)
				left_light = 1;
			else
				left_light = 0;
			while((msgBuf[5]|0x7F)==0x7f)
				usleep(1000);
		}
	
		if((msgBuf[5]|0xDF)==0xdf)
		{
			if (right_light == 0)
				right_light = 1;
			else
				right_light = 0;
			while((msgBuf[5]|0xDF)==0xdf)
				usleep(10000);
		}

		if((msgBuf[6]|0xFB)==0xfb)
		{
			if (hand_brake == 0)
				hand_brake = 1;
			else
				hand_brake = 0;
			while((msgBuf[6]|0xFB)==0xfb)
				usleep(10000);
		}
		else if ((msgBuf[6] |0xFE)==0xfe)
		{
			if (hand_brake == 0)
			{
				if (D_change == 0)
					D_change = 1;
				else
					D_change = 0;
				while((msgBuf[6] |0xFE)==0xfe)
					usleep(10000);
			}
		}
		if (msgBuf[9] < 130 && msgBuf[9]>125)
		{
			steer = 0;
		}
		else
		{
			steer = -(1.0*(msgBuf[9] - 128) / 128) * 200;
		}
		
		if (hand_brake == 0 && D_change == 1)
		{
			if (msgBuf[8] < 130 && msgBuf[8]>125)
			{
				speed = 0; brake = 0;
			}
			else
			{
				if (msgBuf[8] >= 130)
				{
					brake = (1.0*(msgBuf[8] - 128) / 128) * 40;
					speed = 0;
				}
				else if(msgBuf[8]<=125)
				{ 
					speed = (1.0*(128 - msgBuf[8]) / 128) * 15;
					brake = 0;
				}
			}
		}
		if((msgBuf[5]|0xbf) ==0xbf)
			emergencyBrake=1;
		else
			emergencyBrake=0;
///////////////////////////////////////////////////////////////////////////////////////
			
		cmd1.set_horn = didi; 
		cmd1.set_handBrake = hand_brake; 
		cmd1.set_turnLight_R = right_light; 
		cmd1.set_turnLight_L = left_light; 
		cmd1.set_driverlessMode = mode;
		
		cmd2.set_gear = D_change; 
		cmd2.set_brake = brake; 
		cmd2.set_speed = speed; 
		cmd2.set_steeringAngle = steer; 
		cmd2.set_emergencyBrake = emergencyBrake;
		
		pub1.publish(cmd1);
		pub2.publish(cmd2);
		usleep(30000);
	}	
	
	return 0;
}


uint8_t generate_check_sum(uint8_t *buf,int len)
{
  uint8_t sum=0;
  int i=0;
  for(;i<len;i++)
  {
    sum += *(buf+i);
  }
  return sum;
}


void getMsg()
{
	Serial serial;
    if(!serial.openUp(port_name_.c_str(),9600))
    	return ;
    serial.setOption();
    
	while(ros::ok())
    {   
		
    	int len = serial.recv(rearPtr,12);
    	//cout << "len:" <<len <<endl;
    	rearPtr += len;
    	
    	/**
    	for(int i=0;i<len;i++)
    	{
    		printf("%x\t",recvBuf[i]);
    	}
    	cout <<endl;
    	**/
    	
    	
    	for(;rearPtr-headPtr>=12;headPtr++)
    	{
    		if((*headPtr)==0x66 && (*(headPtr+1)) ==0xcc)
    		{
				if(generate_check_sum(headPtr,11) != headPtr[11])
					continue;
				msgHeader = headPtr;
				
				boost::mutex::scoped_lock lock(mutex_); 
				memcpy(msgBuf,msgHeader,12);
				
				headPtr += 12;
				
				break;
    		}
    	}
    	int n_remaind = rearPtr - headPtr;
    	int n_capacity = recvBuf-rearPtr+BufLen;
    	
    	//printf("n_remaind=%d\t n_capacity=%d\n",n_remaind,n_capacity);
    	
    	if(n_remaind<12)
    	{
    		if(n_capacity<12)
    		{
    			memcpy(recvBuf,headPtr,n_remaind);
    			headPtr = recvBuf;
    			rearPtr = recvBuf + n_remaind;
    		}
    	}
    }
}

