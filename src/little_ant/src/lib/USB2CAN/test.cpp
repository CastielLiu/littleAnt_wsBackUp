#include"can2serial.h"

int main()
{
	CAN_2_SERIAL can2serial;
	std::string port_name_ = "/dev/ttyUSB0";

	if(!can2serial.openPort(port_name_.c_str()))
	{
		printf("open port %s failed",port_name_.c_str());
		return 0;
	}
	
	can2serial.clearCanFilter();
	can2serial.setCanFilter(0x03,0x500,0x7c0);
	can2serial.setCanFilter(0x03,0x500,0x7c0);

	
	
	can2serial.run();
	
	can2serial.configBaudrate(500) ;
	
	STD_CAN_MSG msg;

	while(1)
	{
		usleep(1000);
		if(!can2serial.getMsg(msg)) continue;
		
		printf("ID:%x\n",msg.ID);
		
	}
	return 0;
}
