#ifndef CAN2SERIAL_H_
#define CAN2SERIAL_H_
#include<iostream>
#include<stdint.h>
#include "serial.h"
#include <sstream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

//标准can消息缓存区大小
#define MAX_STD_CAN_MSG_BUF  10


struct STD_CAN_MSG
{
    uint16_t ID;
    uint8_t len;
    uint8_t data[8];
};

struct SERIAL_MSG
{
    unsigned char head[2];
    unsigned char pkg_len[2];
    unsigned char cmd;
    unsigned char frame_type;
    unsigned char frame_id[4];
    unsigned char data_len;
    uint8_t data[8];
    uint8_t checkNum;
};


class CAN_2_SERIAL
{
public:

    CAN_2_SERIAL( );

    virtual ~CAN_2_SERIAL();

    bool openPort(const char* port_);
    void closePort();
    
    void run();
    
    void parse_msg();

    unsigned char receiveCanMsgFromDev();

    int sendStdCanMsg(const STD_CAN_MSG &can_msg);

    bool configBaudrate(int baudrate);
    
    int inquireBaudrate(int port = 0x01);

	bool setCanFilter(uint8_t filterNum,int filterID,int filterMask,uint8_t filterMode=0x00);
	bool clearCanFilter(uint8_t filterNum=0xff); //0xff clear all

    bool getMsg(STD_CAN_MSG & msg);

private:
    Serial serial;
    
    //MAX_GET_COUNT  一次从串口缓冲区读取字节最大值
    //MIN_MSG_LEN  最小消息单元包含字节数
    enum{BUF_LENGTH=400 ,MAX_GET_COUNT=200,MIN_MSG_LEN=7};
    enum BaudRateCofigStatus_t
    {
        BaudRateCofig_Ok=0x00,
        BaudrateCofig_Err1=0x01,
        BaudrateCofig_Err2=0x02,
        BaudrateCofig_Err3=0x03,
        BaudrateCofig_Err4=0x04,
        BaudrateCofig_None =0x05
    };
    
    uint8_t filterSetStatus[14];
    uint8_t filterClearStatus;

    uint8_t buf[BUF_LENGTH];
    uint8_t *buf_rear;
    uint8_t *buf_head;
    int remained_bytes;
    int rec_pkg_len;
    
    std::vector<STD_CAN_MSG> stdCanMsgArray;
    
    BaudRateCofigStatus_t baudRateCofigStatus;
    int currentBaudrate;
    

    bool check(const SERIAL_MSG *serial_msg);
    int sendCmd(uint8_t cmdId,const uint8_t *buf,uint8_t count) ;
    uint8_t generateCheckNum(const uint8_t * ,int );
    
    boost::mutex mutex_;

};

#endif
