#ifndef SERIAL_H
#define SERIAL_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>
#include <unistd.h>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class Serial
{
private:
    boost::system::error_code ec_;
	boost::asio::io_service io_service_;
	serial_port_ptr port_;
	
	int baud_rate_;
	
public:
	 enum flush_type_t
		{
		  flush_receive = TCIFLUSH,
		  flush_send = TCOFLUSH,
		  flush_both = TCIOFLUSH
		};

public:
    Serial();
    ~Serial();
    bool tcflush(flush_type_t type);

    bool openUp(std::string port_name,int baud_rate=460800);
    void closeOff();
    bool setOption();
    int send(const unsigned char *data, int length);
    int recv(unsigned char *data, int length);
    bool isOpen();
    
   
};

#endif
