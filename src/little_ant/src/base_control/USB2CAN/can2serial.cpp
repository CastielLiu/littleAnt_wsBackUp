#include "can2serial.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>

using std::cout;
using std::endl;



CAN_2_SERIAL::CAN_2_SERIAL()
{
	stdCanMsgArray.reserve(MAX_STD_CAN_MSG_BUF);
}

CAN_2_SERIAL::~CAN_2_SERIAL()
{
    this->closePort();
   // std::cout << "closePort "<<std::endl;
}


bool CAN_2_SERIAL::openPort(const char* port_)
{
    if(serial.openUp(port_))
    {
        serial.setOption();//460800
        buf_head = buf;
        buf_rear = buf;
        remained_bytes = 0;
        return 1;
    }
    else
        return 0;
}

void CAN_2_SERIAL::closePort()
{
    if(serial.isOpen())
        serial.closeOff();
}

void CAN_2_SERIAL::run()
{
	boost::thread parse_thread(boost::bind(&CAN_2_SERIAL::parse_msg, this));
}



void CAN_2_SERIAL::parse_msg()
{
    while(serial.isOpen())
	{
		receiveCanMsgFromDev();
		usleep(1000);
	}
}

unsigned char CAN_2_SERIAL::receiveCanMsgFromDev()
{
    int len = serial.recv(buf_rear,MAX_GET_COUNT);

    if(len<=0 )
        return 0;
    else
        buf_rear += len;
    /*
    printf("len = %d\r\n",len);
    
    for(uint8_t * ptr=buf_head;ptr<buf_rear;ptr++)
    	printf("%x\t",*ptr);
    printf("\n\n");*/
    //uint8_t *firstBufHeader = buf_head;

    unsigned char pkgCmd =0;
    
    
	
    //当数据长度大于最小消息长度时即应解析数据
    //当数据长度大于最小消息长度但小于stdCanMsg长度时，不应解析 stdCanMsg goto
    //为防止数据覆盖，即使buf中可以解析出多条消息，也只解析一条，剩于的下次再解析
    for(; (buf_rear-buf_head)>=MIN_MSG_LEN;buf_head++)
    {
        if(*((uint16_t *)buf_head) != 0xCC66)
            continue;
		
        SERIAL_MSG *serial_msg_ptr = (SERIAL_MSG *)buf_head;

        rec_pkg_len = serial_msg_ptr->pkg_len[0]*256 + serial_msg_ptr->pkg_len[1];
        
        /*
		static int SEQ_=0;
		SEQ_++;
		printf("%05d\tfind header  %d  pkg_len:%d\r\n",SEQ_,buf_head-firstBufHeader,rec_pkg_len);
		*/
		
        if((buf_rear-buf_head)<rec_pkg_len+4) //包不完整
        {
        	if(MAX_GET_COUNT > buf+BUF_LENGTH-1-buf_rear) //剩余buf空间不足以存放MAX_GET_COUNT
            	goto Data_mosaic;//数据拼接
        	else
        		break;
        }
        	

        if(false == check(serial_msg_ptr)) //校验失败并不一定是数据传输错误,可能是检测到了伪数据头
        									//因此不能跨字节查找
        {
            //printf("check failed!!\n");
           // printf("buf_head=%x\tbuf=%x\r\n",buf_head,buf);
            continue;
        }

        pkgCmd = serial_msg_ptr->cmd;

        switch(pkgCmd)
        {
            case 0xB1: //recieved msg ID
            	STD_CAN_MSG stdCanMsg;
            	
                stdCanMsg.ID = (((uint16_t)(serial_msg_ptr->frame_id[2]))<<8) + (serial_msg_ptr->frame_id[3]);
                
                memcpy(stdCanMsg.data,(buf_head+11),*(buf_head+10));//dst ,src,len
                stdCanMsg.len = *(buf_head+10);
                
                if(stdCanMsgArray.size()==stdCanMsgArray.capacity())
                	stdCanMsgArray.erase(stdCanMsgArray.begin());
                stdCanMsgArray.push_back(stdCanMsg);
                //std::cout << "vectorSize:"<<stdCanMsgArray.size()<< std::endl;
                
                //std::cout<< std::hex <<stdCanMsg.ID<<std::endl;;
                break;

            case 0x92: //baudrate configure status
                baudRateCofigStatus = BaudRateCofigStatus_t(*(buf_head+5));
                break;
                
            case 0x93: //baudrate inquiry response
            	currentBaudrate = *(buf_head+6)*5 ; //kbps
            	break;
            
            case 0x98: //filter set response
            	filterSetStatus[*(buf_head+6)] = *(buf_head+5); //[filterNum]
            	break;
            	
            case 0x99: //clear Filter response
            	filterClearStatus = !(*(buf_head+5));
            	break;

            default:
                break;
        }
        buf_head += (rec_pkg_len+3);  //虽然偏移量为rec_pkg_len+4，但是一定要+3 因为for循环的时候还会+1，
        								//丢包严重才发现这个问题
    }

Data_mosaic:
    int j=0;
    for(;j<(buf_rear-buf_head);j++)
    {
        buf[j] = buf_head[j];
    }

    buf_rear = buf+j;
    buf_head = buf;

    return pkgCmd;
}

int CAN_2_SERIAL::sendStdCanMsg(const STD_CAN_MSG &can_msg)
{
    uint16_t send_pkg_len;

    SERIAL_MSG serial_msg;

    serial_msg.head[0] = 0x66;
    serial_msg.head[1] = 0xCC;

    send_pkg_len = 8 + can_msg.len; //except 2bytes header and 2bytes pkg_len

    serial_msg.pkg_len[0] =  (uint8_t)(send_pkg_len >> 8);
    serial_msg.pkg_len[1] =  (uint8_t)(send_pkg_len);
    serial_msg.cmd = 0x30;
    serial_msg.frame_type = 0x03;
    serial_msg.frame_id[2] = can_msg.ID >>8;
    serial_msg.frame_id[3] = can_msg.ID & 0xFF;
    serial_msg.data_len = can_msg.len;

    for(int i=0;i<can_msg.len;i++)
        serial_msg.data[i] = can_msg.data[i];
    if(can_msg.len == 8)
        serial_msg.checkNum = generateCheckNum((uint8_t *)&serial_msg,send_pkg_len);
    else
        serial_msg.data[can_msg.len] = generateCheckNum((uint8_t *)&serial_msg,send_pkg_len);

    return serial.send((unsigned char*)(&serial_msg),send_pkg_len+4);
}



int CAN_2_SERIAL::sendCmd(uint8_t cmdId,const uint8_t *buf,uint8_t count)
{
    uint8_t send_pkg_len = count + 2;//cmd + checknum + count

    uint8_t * sendBuf = new uint8_t[send_pkg_len +4]; //add head bytes

    sendBuf[0] = 0x66;
    sendBuf[1] = 0xCC;
    sendBuf[2] = (uint8_t)(send_pkg_len >> 8) ;
    sendBuf[3] = (uint8_t)(send_pkg_len);
    sendBuf[4] = cmdId;
    memcpy(&sendBuf[5],buf,count);
    sendBuf[send_pkg_len+3] = generateCheckNum(sendBuf,send_pkg_len);

    int status =  serial.send(sendBuf,send_pkg_len+4);

    delete [] sendBuf;

    return status;
}

bool CAN_2_SERIAL::configBaudrate(int baudrate)
{
    uint8_t buf[2];
    buf[0] = 0x01;//port1
    buf[1] = (baudrate/5)&0xff;
    sendCmd(0x12,buf,2);
    
    usleep(100000);
    
    bool status=0;
    
    if(baudRateCofigStatus==BaudRateCofig_Ok)
    {
    	status = 1;
    	baudRateCofigStatus = BaudrateCofig_None;
    }
	return status;
}

bool CAN_2_SERIAL::setCanFilter(uint8_t filterNum,int filterID,int filterMask,uint8_t filterMode)
{
	uint8_t buf[11];
	buf[0] = 0x01; //port
	buf[1] = filterNum;
	filterID <<= 21;
	filterMask <<=21;
	if(filterMode==0x00)
	{
		buf[2] = (filterID >> 24)&0xff;
		buf[3] = (filterID >>16)&0xff;
		buf[4] = 0x00;
		buf[5] = 0x00;
		
		buf[6] = (filterMask >> 24)&0xff;
		buf[7] = (filterMask >> 18)&0xff;
		buf[8] = 0x00;
		buf[9] = 0x00;
		
	}
	buf[10] = filterMode;
	sendCmd(0x18,buf,11);
	
	usleep(100000);
	bool status = 0;
	if(filterSetStatus[filterNum] == 0x00)
	{
		status =1;
		filterSetStatus[filterNum] = 0x05;
	}
	
	return status;
}

bool CAN_2_SERIAL::clearCanFilter(uint8_t filterNum)
{
	uint8_t buf[2];
	buf[0] = 0x01;//port
	buf[1] = filterNum;
	sendCmd(0x19,buf,2);
	
	usleep(100000);
	
	if(filterClearStatus==1)
	{
		filterClearStatus =0;
		return 1;
	}
	else
		return 0;
	
}


int CAN_2_SERIAL::inquireBaudrate(int port)
{
	uint8_t buf[1];
	if(port==1)
		buf[0] = 0x01;
	else if(port==2)
		buf[0] = 0x02;
	else
		return 0;
		
	sendCmd(0x13,buf,1);
	
	usleep(100000);//100ms

	return this->currentBaudrate;
}

bool CAN_2_SERIAL::getMsg(STD_CAN_MSG &msg)
{
	if(stdCanMsgArray.empty())
		return 0;
	msg = stdCanMsgArray[0];
	stdCanMsgArray.erase(stdCanMsgArray.begin());
	
	return 1;
}

bool CAN_2_SERIAL::check(const SERIAL_MSG *serial_msg)
{
    const unsigned char *ptr = (unsigned char *)serial_msg;
    if(generateCheckNum(ptr,rec_pkg_len) == *(ptr+4+rec_pkg_len-1))
        return true;
    else
        return false;
}

uint8_t CAN_2_SERIAL::generateCheckNum(const uint8_t* ptr,int _pkg_len_)
{
    int temp=0;

    for(int i=2; i<_pkg_len_+3; i++)
    {
        temp += ptr[i];
    }
    return (unsigned char)(temp&0xff);
}

