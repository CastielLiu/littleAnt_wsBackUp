#include<boost/thread.hpp>
#include<boost/bind.hpp>

#include<iostream>
#include"serial.h"

class GetPkgFromDev
{
	public:
		GetPkgFromDev(std::string port_name,int baudrate,int pkg_length):
			port_name_(port_name),
			baudrate_(baudrate),
			pkg_length_(pkg_length),
			recvBufLen_(pkg_length_*10)
		{
			pkgBuf = new uint8_t[pkg_length_];
			recvBuf = new uint8_t[recvBufLen_];
			
			headPtr = recvBuf;
			rearPtr = recvBuf;
			msgHeader =NULL;
		}
		bool init()
		{
			if(!serial.openUp(port_name_.c_str(),baudrate_))
    			return 0;
    		serial.setOption();
    		
    		return 1;
		}
		void startTread()
		{
			boost::thread get_(boost::bind(&GetPkgFromDev::read_and_data_mosaicing,this));
		}
		~GetPkgFromDev()
		{
			delete [] pkgBuf;
			delete [] recvBuf;
			pkgBuf =NULL;
			recvBuf =NULL;
			if(serial.isOpen())
				serial.closeOff();
		}
		const uint8_t *getPkgPtr()
		{
			return pkgBuf;
		}
		
		void closeThread()
		{
			getThread_flag=false;
		}
		
	private:
		void read_and_data_mosaicing()
		{
			getThread_flag = true;
			while(getThread_flag)
			{   
				int len = serial.recv(rearPtr,pkg_length_);
				
			/*	for(int i=0;i<len;i++)
				{
					printf("%x\t",rearPtr[i]);
				}
				printf("\n");
			*/	
				rearPtr += len;
				
				for(;rearPtr-headPtr>=pkg_length_;headPtr++)
				{
					
					if((*headPtr)==0x66 && (*(headPtr+1)) ==0xcc)
					{
						if(generate_check_sum(headPtr,pkg_length_-1) != headPtr[pkg_length_-1])
							continue;
						msgHeader = headPtr;
				
						boost::mutex::scoped_lock lock(mutex_); 
						memcpy(pkgBuf,msgHeader+2,pkg_length_-2);
				
						headPtr += pkg_length_;
				
						break;
					}
				}
				int n_remaind = rearPtr - headPtr;
				int n_capacity = recvBuf-rearPtr+recvBufLen_;
			
				//printf("n_remaind=%d\t n_capacity=%d\n",n_remaind,n_capacity);
			
				if(n_remaind<pkg_length_)
				{
					if(n_capacity<pkg_length_)
					{
						memcpy(recvBuf,headPtr,n_remaind);
						headPtr = recvBuf;
						rearPtr = recvBuf + n_remaind;
					}
				}
			}
		}

		uint8_t generate_check_sum(uint8_t *buf,int len)
		{
			uint8_t sum=0;
			int i=0;
			for(;i<len;i++)
				sum += *(buf+i);
			
			return sum;
		}
	
	private:
		std::string port_name_;
		int baudrate_;
		uint8_t *recvBuf;
		int pkg_length_;
		int recvBufLen_;
		uint8_t *pkgBuf;
		boost::mutex mutex_;
		Serial serial;
		bool getThread_flag;
		

		uint8_t *msgHeader;
		uint8_t *headPtr;
		uint8_t *rearPtr;
		
};

