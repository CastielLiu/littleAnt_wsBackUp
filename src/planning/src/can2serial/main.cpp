#include"can2serial.h"

int main()
{

	Can2serial can2serial;
	can2serial.configure_port("/dev/ttyUSB0",460800);
	can2serial.StartReading();
	CanMsg_t can_msg;
	
	usleep(100000);
	can2serial.clearCanFilter();
	can2serial.configBaudrate(250);
	
	//can2serial.setCanFilter(0x01,0x123,0x7ff);
	
	
	while(1)
	{
		usleep(1000);
		if(!can2serial.getCanMsg(can_msg))
			continue;
		printf("ID:%x\n",can_msg.ID);
		can2serial.sendCanMsg(can_msg);
	}
	
	return 0;
}
