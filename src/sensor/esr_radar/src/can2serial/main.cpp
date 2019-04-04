#include"can2serial.h"
#include<ros/ros.h>

int main()
{

	Can2serial can2serial;
	can2serial.configure_port("/dev/ttyUSB2",460800);
	can2serial.StartReading();
	CanMsg_t can_msg;
	
	usleep(100000);
	can2serial.clearCanFilter();
	can2serial.clearCanFilter();
	can2serial.configBaudrate(500);
	
	can2serial.setCanFilter_alone(0x01,0x4E0); usleep(1000);
	can2serial.setCanFilter_alone(0x02,0x4E1); usleep(1000);
	//can2serial.setCanFilter(0x02,0x500,0x7c0); //500-53f
	
	//can2serial.setCanFilter(0x01,0x123,0x7ff);
	
	
	while(1)
	{
		//usleep(1000);
		if(!can2serial.getCanMsg(can_msg))
			continue;
		ROS_INFO("ID:%x",can_msg.ID);
		//can2serial.sendCanMsg(can_msg);
	}
	
	return 0;
}
