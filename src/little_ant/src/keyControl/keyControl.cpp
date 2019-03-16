#include<iostream>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>
#include"little_ant_msgs/ControlCmd1.h"
#include"little_ant_msgs/ControlCmd2.h"

#include<ros/ros.h>


using namespace std;

int main(int argc ,char **argv)
{
	int keys_fd ,mouse_fd;
    char ret[2];
    struct input_event keyEvent,mouseEvent;
    keys_fd=open("/dev/input/event1",O_RDONLY|O_NONBLOCK);//只读&非阻塞MODE
  //  mouse_fd=open("/dev/input/event2",O_RDONLY|O_NONBLOCK);//只读&非阻塞 MODE
    if(keys_fd<=0)
    {   
        printf("error\n");
        return -1; 
    }   
    
	ros::init(argc,argv,"key_control");
	
	ros::NodeHandle nh;
	
	ros::Publisher pub1 = nh.advertise<little_ant_msgs::ControlCmd1>("/controlCmd1",10);
	ros::Publisher pub2 = nh.advertise<little_ant_msgs::ControlCmd2>("/controlCmd2",10);
	
	double speed=0, steer=0, steer_right=0, brake=0;
	int didi=0, left_light=0, right_light=0, D_change=0, hand_brake=0;
	int keyboard=65535;
	little_ant_msgs::ControlCmd1 cmd1;
	little_ant_msgs::ControlCmd2 cmd2;
	
	
    while(1)
    {   
   		pub1.publish(cmd1);
		pub2.publish(cmd2);
    	usleep(1000);
        read(keys_fd,&keyEvent,sizeof(struct input_event));
       // read(mouse_fd,&mouseEvent,sizeof(struct input_event));
        if(keyEvent.type==1 && keyEvent.value==1)
        {
        	// printf(" value %i type=%d \t code =%d\n", keyEvent.value,keyEvent.type,keyEvent.code);
        	keyboard = keyEvent.code;
        }
        else
        	continue;
           
            
		
		//std::cout << keyboard << std::endl;
		if(keyboard==57)
		{cmd1.set_driverlessMode=1;}
			if (keyboard == 45)//脌庐掳脠
		{
			if (didi == 0)
				didi = 1;
			else
				didi = 0;
		}
		else if(keyboard == 44)//脳贸脳陋脧貌碌脝
		{
			if (left_light == 0)
				left_light = 1;
			else
				left_light = 0;
		}
		if (keyboard == 46)//脫脪脳陋脧貌碌脝
		{
			if (right_light == 0)
				right_light = 1;
			else
				right_light = 0;
		}
		else if(keyboard == 48)//b
		{
			if (hand_brake == 0)
				hand_brake = 1;
			else
				hand_brake = 0;
		}
		else if(keyboard==47)	
		{
			if (hand_brake == 0)
			{
				if (D_change == 0)
					D_change = 1;
				else
					D_change = 0;
			}
		}
		
		 
		if (hand_brake == 1 || D_change == 0)//脠路露拧脢脰脡虏脣脡碌么脟脪鹿脪D碌碌
		{
			speed = 0;
		}
			
		if (hand_brake == 0 && D_change == 1)
		{
			if (brake == 0)
			{
				if (keyboard == 103)//up
				{
					if (speed + 1 > 15)
						speed = 15;
					else
						speed = speed + 1;
				}
				if (keyboard == 17)//艗玫脣脵
				{
					if (speed - 5 < 0)
						speed = 0;
					else
						speed = speed -5;
				}
				if (keyboard == 16) //q
					speed = 0;
				
			}
			if (keyboard == 18)
			{
				steer = 0;
			}
			if (keyboard == 105)//脝脮脥拧脳贸脳陋
			{
				if (steer - 20 < -500)
					steer = -500;
				else
					steer = steer - 20;
			}
			if (keyboard == 30)//驴矛脣脵脳贸脳陋
			{
				if (steer - 100 < -500)
					steer = 500;
				else
					steer = steer - 100;
			}
			if (keyboard == 106)//脝脮脥拧脫脪脳陋
			{
				if (steer + 20 > 500)
					steer = 500;
				else
					steer = steer + 20;
			}
			if (keyboard == 32)//驴矛脣脵脫脪脳陋
			{
				if (steer + 100 > 500)
					steer = 500;
				else
					steer = steer + 100;
			}

			if (keyboard == 108)// down
			{
				speed = 0;
				if (brake + 1 > 40)
					brake = 40;
				else
					brake = brake + 4;
			}
			if (keyboard == 31)//s
			{
				speed = 0;
				if (brake - 1 < 0)
					brake = 0;
				else
					brake = brake - 4;
			}
			if (keyboard == 49)//脡虏鲁碌啪沤脦禄
			{
				brake = 0;
			}		
		}
		
		//cout << speed <<" " << steer << " " << steer_right << " " << brake << " " << D_change <<" " << hand_brake << "\n";
		cmd1.set_horn = didi;//À®°Èx
		cmd1.set_handBrake = hand_brake;//ÊÖÉ²b
		cmd1.set_turnLight_R = right_light;//ÓÒ×ªÏòµÆc
		cmd1.set_turnLight_L = left_light;//×ó×ªÏòµÆz
		cmd2.set_gear = D_change;//µµÎ»v
		cmd2.set_brake = brake;//É²³µs/žŽÎ»n
		cmd2.set_speed = speed;//ËÙ¶Èw/ŒõËÙW/žŽÎ»q
		cmd2.set_steeringAngle = steer;//×ªÏò ×óaÓÒd/žŽÎ»e
		
		
		usleep(300000);
		if(keyboard==1)
		{
			close(keys_fd);
			return 0;
		}
			
	}
		//ros::spin();
	close(keys_fd);
	return 0;
}
