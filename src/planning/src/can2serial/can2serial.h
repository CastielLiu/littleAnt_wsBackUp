#ifndef CAN_2_SERIAL_H__
#define CAN_2_SERIAL_H__

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "serial/serial.h"
#include <vector>
#include<iostream>
#include<string>

#define MAX_NOUT_SIZE  200

typedef struct
{
	uint8_t type;
    uint16_t ID;
    uint8_t len;
    uint8_t data[8];
}CanMsg_t;

class Can2serial
{
public:
	Can2serial();
	~Can2serial();
	
	bool configure_port(std::string port,int baud_rate);

	void StartReading() ;

	void StopReading() ;

	bool sendCanMsg(const CanMsg_t &can_msg);
	
	bool configBaudrate(int baudrate);

	bool setCanFilter(uint8_t filterNum,int filterID,int filterMask,uint8_t filterMode=0x00);

	bool setCanFilter_alone(uint8_t filterNum,int filterID,uint8_t filterMode=0x00);

	bool clearCanFilter(uint8_t filterNum=0xff);

	int inquireBaudrate(uint8_t port =0x01);

	bool getCanMsg(CanMsg_t &msg);
	
	
private:


	void ReadSerialPort() ;

	void BufferIncomingData(unsigned char *message, unsigned int length);

	void parse(uint8_t * message,uint16_t length);

	uint8_t generateCheckNum(const uint8_t* ptr,size_t len);

	void sendCmd(uint8_t cmdId,const uint8_t *buf,uint8_t count);

	serial::Serial *serial_port_;
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;
	
	uint8_t data_buffer_[20]; //最大包长为20
	size_t buffer_index_;
	
	uint16_t package_len_;
	size_t bytes_remaining_;
	
	CanMsg_t canMsg_;
	std::vector<CanMsg_t> canMsgArray_;
	
	boost::mutex mutex_;
	
	
	
	enum{HeaderByte0=0x66,HeaderByte1=0xCC};
	enum
	{
		CanMsgCmd = 0xB1,
		BaudrateInquireResponseCmd = 0x93,
		FilterSetResponseCmd = 0x98,
		FilterClearResponseCmd = 0x99
	};
	
	enum BaudRateCofigStatus_t
    {
        BaudRateCofig_Ok=0x00,
        BaudrateCofig_Err1=0x01,
        BaudrateCofig_Err2=0x02,
        BaudrateCofig_Err3=0x03,
        BaudrateCofig_Err4=0x04,
        BaudrateCofig_None =0x05
    };
    
    BaudRateCofigStatus_t baudRateCofigStatus_;
    int currentBaudrate_;
    
    bool filterSetStatus_[14];
    bool filterClearStatus_;

};



 







#endif


