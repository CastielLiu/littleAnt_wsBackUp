#include <iostream>
#include "serial.h"
#include <unistd.h>

using namespace std;

Serial::Serial() 
{
    
}

Serial::~Serial()
{
	closeOff();
}

bool Serial::openUp(std::string port_name,int baud_rate)
{
	baud_rate_ = baud_rate;
	
	if (port_) 
	{
	    perror("error : port is already opened...");
	    return false;
	}
	port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
	port_->open(port_name, ec_);
	if (ec_) 
	{
	    //printf("error : port_->open() failed...port_name=" << port_name_ << ", e=" << ec_.message().c_str());
		return false;
	}
  
  return true;
  	
}

void  Serial::closeOff()
{
    if (port_) 
	{
	    port_->cancel();
	    port_->close();
	    port_.reset();
    }
	io_service_.stop();
	io_service_.reset();
}

bool Serial::setOption( )
{

	// option settings...
  port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
  port_->set_option(boost::asio::serial_port_base::character_size(8));
  port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

	return true;
}


int Serial::send(const unsigned char *data, int length)
{
    return  boost::asio::write(*port_.get(), boost::asio::buffer(data, length), ec_);
}

int Serial::recv(unsigned char *data, int length)
{
    return boost::asio::read(*port_.get(), boost::asio::buffer(data, length), ec_);
}


bool Serial::isOpen()
{
	if(port_)
		return true;
	else
		return false;
}

