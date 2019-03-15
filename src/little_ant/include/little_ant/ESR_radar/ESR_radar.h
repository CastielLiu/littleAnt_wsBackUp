#ifndef ESR_RADAR_H_
#define ESR_RADAR_H_
#include "can2serial.h"
 
#define RECEIVE__ 0
#define SEND__  1


class ESR_RADAR 
{
	public:
		 ESR_RADAR();
		~ESR_RADAR();
		
		bool init(const char* port_, bool mode) ;
		
		
		void run();
		void handleCanMsg();
		
		void pub_installHeight(uint8_t installHeight=10);
		void pub_vehicleSpeed(float speed, bool dir=0);
		
	private:
		CAN_2_SERIAL *can2serial;
		
		void parse_msg(STD_CAN_MSG &can_msg);
		
		
};

#endif
