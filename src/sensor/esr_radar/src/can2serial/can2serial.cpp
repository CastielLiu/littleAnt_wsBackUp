#include "can2serial.h"
#include<cstring>


Can2serial::Can2serial() :
	inquire_filter_response_ptr_((inquireFilterResponsePkg_t *)data_buffer_)
{
	serial_port_=NULL;
	reading_status_=false;
	buffer_index_ = 0;
	bytes_remaining_ = 0;
	baudRateCofigStatus_ =BaudrateCofig_None;
	filterClearStatus_ = false;
	for(size_t i=0;i<MAX_MSG_BUF_SIZE;i++)
		canMsgStatus[i] = false;
	readIndex_ =0;
	writeIndex_ =0;
}

Can2serial::~Can2serial()
{
	this->StopReading();
	serial_port_->close();
	if(serial_port_!=NULL)
	{
		delete serial_port_;
		serial_port_ = NULL;
	}
}

bool Can2serial::configure_port(std::string port,int baud_rate)
{
	try 
	{
	
		serial_port_ = new serial::Serial(port,baud_rate,serial::Timeout::simpleTimeout(10)); 

		if (!serial_port_->isOpen())
		{
	        std::stringstream output;
	        output << "Serial port: " << port << " failed to open." << std::endl;
			delete serial_port_;
			serial_port_ = NULL;
			return false;
		} 
		else 
		{
	        std::stringstream output;
	        output << "Serial port: " << port << " opened successfully." << std::endl;
	        std::cout << output.str() <<std::endl;
		}

		serial_port_->flush();
		
	} 
	catch (std::exception &e) 
	{
	    std::stringstream output;
	    output << "Error  " << port << ": " << e.what();
	    std::cout << output.str() <<std::endl;
	    return false;
	}

	return true;
}


void Can2serial::StartReading() 
{
	if (reading_status_)
		return;
		
	//serial_port_->flush();
	// create thread to read from sensor
	reading_status_=true;
	read_thread_ptr_ = boost::shared_ptr<boost::thread >(new boost::thread(boost::bind(&Can2serial::ReadSerialPort, this)));
}

void Can2serial::StopReading() 
{
	reading_status_=false;
}
 
void Can2serial::ReadSerialPort() 
{
	size_t len;

	// continuously read data from serial port
	while (reading_status_) 
	{
		usleep(10000);
		try 
		{
			// read data
			len = serial_port_->read(buffer, MAX_NOUT_SIZE);
		} 
		catch (std::exception &e) 
		{
	        std::stringstream output;
	        output << "Error reading from serial port: " << e.what();
	        std::cout << output.str() <<std::endl;
    	}
    	if(len==0) continue;
    	
    	
		// add data to the buffer to be parsed
		
		//ROS_INFO("read length :%d\r\n",len);
		BufferIncomingData(buffer, len);
		
		
		std::cout << std::dec<< "length:" <<len <<std::endl;
		/*for(size_t i=0;i<len;i++)
			printf("%x\t",buffer[i]);
		printf("\n\n");
		*/
	}
	
}


		 
          
void Can2serial::BufferIncomingData(unsigned char *message, unsigned int length)
{
	// add incoming data to buffer
	for (unsigned int ii=0; ii<length; ii++) 
	{// make sure bufIndex is not larger than buffer
		if(buffer_index_>=MAX_PKG_BUF_LEN)
		{
			buffer_index_ = 0;
			  printf("Overflowed receive buffer. Buffer cleared.");
		}
		//std::cout << "bytes_remaining_...:" <<bytes_remaining_  <<std::endl;
		switch(buffer_index_)
		{
			case 0: //nothing
				if(message[ii]==HeaderByte0)
				{
					data_buffer_[buffer_index_++]=message[ii];
				}
				bytes_remaining_ = 0;
				break;
			case 1:
				if(message[ii]==HeaderByte1)
				{
					data_buffer_[buffer_index_++]=message[ii];
					bytes_remaining_ =2;
				}
				else
				{
					buffer_index_=0;
					bytes_remaining_ =0;
				}
				break;
			case 2:
			case 3:
				data_buffer_[buffer_index_++]=message[ii];
				bytes_remaining_ --;
				if(bytes_remaining_==0)
				{
					package_len_ = (data_buffer_[buffer_index_-2] << 8)+data_buffer_[buffer_index_-1] ;
					bytes_remaining_ = package_len_;
					//printf("\t%x\t%x\n",data_buffer_[buffer_index_-2],data_buffer_[buffer_index_-1]);
				}
				break;
			default:
				data_buffer_[buffer_index_++]=message[ii];
				bytes_remaining_--;
				if(bytes_remaining_==0)
				{
				 //std::cout << "package_len_...:" <<package_len_ << std::endl;
					parse(data_buffer_,package_len_+4);
					buffer_index_ = 0;
					
				}
				break;
			
		
		}
	}	// end for
}

void Can2serial::parse(uint8_t * message,uint16_t length)
{
	if(generateCheckNum(message,length)!= message[length-1])
	{
		printf("check failed !!!\r\n");
		return;
	}
	
	uint8_t cmd = data_buffer_[4];
	//printf("cmd:%x\n",cmd);
	switch(cmd)
	{
		case CanMsgCmd:
            	
            	//if(0==(data_buffer_[12] >>5)) break;                           //data[1] esr_radar object status
            																//just used in esr_radar to filter invalid objects
            	canMsg_.type = data_buffer_[5];
                canMsg_.ID = (data_buffer_[6]<<24)+(data_buffer_[7]<<16)+(data_buffer_[8]<<8)+(data_buffer_[9]);
                canMsg_.len = data_buffer_[10];
                
                //for(size_t i=0;i<canMsg_.len;i++)
                //	canMsg_.data[i] = data_buffer_[11+i];
                
                memcpy(canMsg_.data,data_buffer_+11,canMsg_.len);
                
                
                {	
                	boost::mutex::scoped_lock lock(mutex_);
		            canMsgBuf_[writeIndex_] = canMsg_;
		            canMsgStatus[writeIndex_] = true;
		        }
                writeIndex_++;
                if(writeIndex_==MAX_MSG_BUF_SIZE)
                	writeIndex_=0;
                
                //std::cout<< std::hex <<stdCanMsg.ID<<std::endl;;
                break;
		
		case FilterClearResponseCmd:
			filterClearStatus_ = !data_buffer_[5];
			
			break;
		
		case FilterSetResponseCmd:
			filterSetStatus_[data_buffer_[6]] = !data_buffer_[5];
			
			break;
			
		case BaudrateInquireResponseCmd:
			currentBaudrate_ = data_buffer_[6]*5;
			break;
			
		case InquireFilterResponse:
			if((ntohl(inquire_filter_response_ptr_->filterID)>>21)!=0)
				printf("filter_num:%x\t filterID:%x\t filterMask:%x\r\n",
					inquire_filter_response_ptr_->filterNum,ntohl(inquire_filter_response_ptr_->filterID)>>21,
					ntohl(inquire_filter_response_ptr_->filterMask)>>21);
			break;
			
		//case FilterInquireResponseCmd;
			
		
		default:
			break;

		
	}
}

uint8_t Can2serial::generateCheckNum(const uint8_t* ptr,size_t len)
{
    uint8_t sum=0;

    for(int i=2; i<len-1 ; i++)
    {
        sum += ptr[i];
        //printf("%x\t",ptr[i]);
    }
   // printf("-----------\n");
    return sum;
}

bool Can2serial::sendCanMsg(const CanMsg_t &can_msg)
{
	uint16_t send_pkg_len = 12 + can_msg.len;
	
	uint8_t *sendBuf = new uint8_t;
	
	sendBuf[0] = 0x66;
	sendBuf[1] = 0xCC;
	
	uint16_t length = send_pkg_len-4;
	sendBuf[2] = length >>8;
	sendBuf[3] = length;
	
	sendBuf[4] = 0x30;
	sendBuf[5] = can_msg.type;
	sendBuf[6] = can_msg.ID >>24;
	sendBuf[7] = can_msg.ID >>16;
	sendBuf[8] = can_msg.ID >>8;
	sendBuf[9] = can_msg.ID;
	
	sendBuf[10] = can_msg.len;
	
	for( size_t i=0; i<can_msg.len;i++)
		sendBuf[11+i] = can_msg.data[i];
		
	sendBuf[send_pkg_len-1] = generateCheckNum(sendBuf,send_pkg_len);

	try
	{
		serial_port_->write(sendBuf,send_pkg_len);
		delete [] sendBuf;
	}
    	
	catch (std::exception &e) 
	{
		std::stringstream output;
		output << "Error changing baud rate: " << e.what();
		std::cout << output.str() <<std::endl;
		return false;
	}
	return true;
}



void Can2serial::sendCmd(uint8_t cmdId,const uint8_t *buf,uint8_t count)
{
    uint8_t send_pkg_len = count + 2;//cmd + checknum + count

    uint8_t * sendBuf = new uint8_t[send_pkg_len +4]; //add head bytes

    sendBuf[0] = 0x66;
    sendBuf[1] = 0xCC;
    sendBuf[2] = (uint8_t)(send_pkg_len >> 8) ;
    sendBuf[3] = (uint8_t)(send_pkg_len);
    sendBuf[4] = cmdId;
    memcpy(&sendBuf[5],buf,count);
    sendBuf[send_pkg_len+3] = generateCheckNum(sendBuf,send_pkg_len+4);

    serial_port_->write(sendBuf,send_pkg_len+4);
    
    /*
    for(int i =0;i<send_pkg_len+4;i++)
    	printf("%x\t",sendBuf[i]);
    printf("\n");
*/
    delete [] sendBuf;
}

bool Can2serial::configBaudrate(int baudrate)
{
    uint8_t buf[2];
    buf[0] = 0x01;//port1
    buf[1] = (baudrate/5)&0xff;
    sendCmd(0x12,buf,2);
    
    usleep(100000);
    
    sendCmd(0x12,buf,2);
    
    bool status=false;
    
    if(baudRateCofigStatus_==BaudRateCofig_Ok)
    {
    	status = true;
    	baudRateCofigStatus_ = BaudrateCofig_None;
    }
	return status;
}

bool Can2serial::setCanFilter(uint8_t filterNum,int filterID,int filterMask,uint8_t filterMode)
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
		buf[7] = (filterMask >> 16)&0xff;
		buf[8] = 0x00;
		buf[9] = 0x00;
		
	}
	buf[10] = filterMode;
	sendCmd(0x18,buf,11);
	
	usleep(100000);
	
	bool status = false;
	if(filterSetStatus_[filterNum] == true)
	{
		status =true;
		filterSetStatus_[filterNum] = false;
	}
	
	return status;
}

bool Can2serial::setCanFilter_alone(uint8_t filterNum,int filterID,uint8_t filterMode)
{

	uint8_t buf[11];
	buf[0] = 0x01; //port
	buf[1] = filterNum;
	filterID <<= 21;

	buf[2] = (filterID >> 24)&0xff;
	buf[3] = (filterID >>16)&0xff;
	buf[4] = 0x00;
	buf[5] = 0x00;
		
	buf[6] = 0xff;
	buf[7] = 0xff;
	buf[8] = 0xff;
	buf[9] = 0xff;
		

	buf[10] = filterMode;
	sendCmd(0x18,buf,11);
	
	usleep(100000);
	
	sendCmd(0x18,buf,11);
	
	if(filterSetStatus_[filterNum]==true)
		return true;
	else
		return false;
}

bool Can2serial::clearCanFilter(uint8_t filterNum)
{
	uint8_t buf[2];
	buf[0] = 0x01;//port
	buf[1] = filterNum;
	sendCmd(0x19,buf,2);
	
	usleep(100000);
	
	sendCmd(0x19,buf,2);
	
	if(filterClearStatus_==true)
	{
		filterClearStatus_ =false;
		return true;
	}
	else
		return false;
	
}


void Can2serial::inquireBaudrate(uint8_t port)
{
	uint8_t buf[1];
	if(port==1)
		buf[0] = 0x01;
	else if(port==2)
		buf[0] = 0x02;
	else
		return ;
		
	sendCmd(0x13,buf,1);
	
	usleep(10000);//100ms

}

bool Can2serial::getCanMsg(CanMsg_t &msg)
{
	
	if(canMsgStatus[readIndex_]==false || readIndex_==writeIndex_)
		return false;
	{	
		boost::mutex::scoped_lock lock(mutex_);
		msg = canMsgBuf_[readIndex_];
		canMsgStatus[readIndex_]=false;
	}
	readIndex_++;
	if(readIndex_==MAX_MSG_BUF_SIZE)
		readIndex_= 0;
	
	return true;
}


void Can2serial::inquireFilter(uint8_t filterNum ,uint8_t port)
{
	uint8_t buf[2];
	if(port==1)
		buf[0] = 0x01;
	else if(port==2)
		buf[0] = 0x02;
	else
		return ;
	buf[1] = filterNum;
		
	sendCmd(0x1D,buf,2);
	
	usleep(10000);
}







