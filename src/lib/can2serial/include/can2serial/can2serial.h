#ifndef CAN_2_SERIAL_H__
#define CAN_2_SERIAL_H__

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "serial/serial.h"
#include <vector>
#include<iostream>
#include<string>

#include <arpa/inet.h> 

#define MAX_NOUT_SIZE  2000  //read from serial per time
#define MAX_MSG_BUF_SIZE 200  //complete can msg max capacity  ring storage area
#define MAX_PKG_BUF_LEN 50   // can2serial max pkg len  >20 to avoid data overflow

#ifdef _MSC_VER // using MSVC
	#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
	#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif

static unsigned char buffer[MAX_NOUT_SIZE];

typedef struct
{
    uint16_t ID;
    uint8_t len;
    uint8_t data[8];
    uint8_t type; //stdFrame + dataFrame
}CanMsg_t;

PACK(
struct stdCanMsgPkg_t {
    uint8_t header1; //66
    uint8_t header2; //cc
    uint16_t pkg_len;
    uint8_t pkg_cmd; //B1 stdCanMsg
    uint8_t type; //frame type
    uint32_t frame_id;
    uint8_t frame_len;
    uint8_t data[8];
    uint8_t checknum;
});

PACK(
struct inquireFilterResponsePkg_t {
    uint8_t header1; //66
    uint8_t header2; //cc
    uint16_t pkg_len;
    uint8_t pkg_cmd; //9D inquire Filter Response 
    uint8_t result;
    uint8_t portNum;
    uint8_t filterNum;
    uint32_t filterID;
    uint32_t filterMask;
    uint8_t filterMode;
    uint8_t checknum;
});



class Can2serial
{
public:
	Can2serial();
	~Can2serial();
	
	bool configure_port(std::string port,int baud_rate=460800);

	void StartReading() ;

	void StopReading() ;

	bool sendCanMsg(const CanMsg_t &can_msg);
	
	bool configBaudrate(int baudrate);

	bool setCanFilter(uint8_t filterNum,int filterID,int filterMask,uint8_t filterMode=0x00);

	bool setCanFilter_alone(uint8_t filterNum,int filterID,uint8_t filterMode=0x00);

	bool clearCanFilter(uint8_t filterNum=0xff);

	void inquireBaudrate(uint8_t port =0x01);

	bool getCanMsg(CanMsg_t &msg);
	
	void inquireFilter(uint8_t filterNum ,uint8_t port=0x01);
	
	
private:


	void ReadSerialPort() ;

	void BufferIncomingData(unsigned char *message, unsigned int length);

	void parse(uint8_t * message,uint16_t length);

	uint8_t generateCheckNum(const uint8_t* ptr,size_t len);

	void sendCmd(uint8_t cmdId,const uint8_t *buf,uint8_t count);

	serial::Serial *serial_port_;
	boost::shared_ptr<boost::thread> read_thread_ptr_;
	bool reading_status_;
	
	uint8_t data_buffer_[MAX_PKG_BUF_LEN]; //最大包长为20
	size_t buffer_index_;
	
	uint16_t package_len_;
	size_t bytes_remaining_;
	
	CanMsg_t canMsg_;
	
	CanMsg_t canMsgBuf_[MAX_MSG_BUF_SIZE];
	bool canMsgStatus[MAX_MSG_BUF_SIZE];
	
	size_t writeIndex_;
	size_t readIndex_;
	
	boost::mutex mutex_;
	
	const inquireFilterResponsePkg_t * const inquire_filter_response_ptr_;
	
	
	
	enum{HeaderByte0=0x66,HeaderByte1=0xCC};
	enum
	{
		CanMsgCmd = 0xB1,
		BaudrateInquireResponseCmd = 0x93,
		FilterSetResponseCmd = 0x98,
		FilterClearResponseCmd = 0x99,
		InquireFilterResponse = 0x9D
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


