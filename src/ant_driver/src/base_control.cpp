#include<ant_driver/base_control.h>
#include<assert.h>

static float g_steering_gearRatio = 540.0/25.0;

static bool openSerial(serial::Serial* & port_ptr, std::string port_name,int baud_rate)
{
	try
	{
		port_ptr = new serial::Serial(port_name,baud_rate,serial::Timeout::simpleTimeout(10)); 

		if (!port_ptr->isOpen())
		{
	        std::stringstream output;
	        output << "Serial port: " << port_name << " failed to open." << std::endl;
			delete port_ptr;
			port_ptr = NULL;
			return false;
		} 
		else 
		{
	        std::stringstream output;
	        output << "Serial port: " << port_name << " opened successfully." << std::endl;
	        std::cout << output.str() <<std::endl;
		}

		port_ptr->flush();
	} 
	catch (std::exception &e) 
	{
	    std::stringstream output;
	    output << "Error  " << port_name << ": " << e.what();
	    std::cout << output.str() <<std::endl;
	    return false;
	}
	
	return true;
}

BaseControl::BaseControl():
	stm32_msg1Ptr_((const stm32Msg1_t *)stm32_pkg_buf)
{
	is_driverlessMode_ = false;
	stm32_serial_port_ = NULL;
	
	canMsg_cmd1.ID = ID_CMD_1;
    canMsg_cmd1.len = 8;
    canMsg_cmd1.type = Can2serial::STD_DATA_FRAME; //standard frame;
    
    *(long *)canMsg_cmd1.data = 0;

    canMsg_cmd2.ID = ID_CMD_2;
    canMsg_cmd2.len = 8;
    canMsg_cmd2.type = Can2serial::STD_DATA_FRAME;//standard frame;
    
    *(long *)canMsg_cmd2.data = 0;
    canMsg_cmd2.data[4] = 0xFF;
    canMsg_cmd2.data[5] = 0xFF; //set the steer angle value invalid
    
}

BaseControl::~BaseControl()
{
	stm32_serial_port_->close();
	if(stm32_serial_port_!=NULL)
	{
		delete stm32_serial_port_;
		stm32_serial_port_ = NULL;
	}
}

bool BaseControl::init(int argc,char**argv)
{
	ros::init(argc,argv,"base_control");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	
	nh_private.param<std::string>("obd_can_port_name", obd_can_port_name_, "");
	nh_private.param<std::string>("stm32_port_name", stm32_port_name_, "");
	nh_private.param<float>("max_steering_speed",max_steering_speed_,2.0);
	nh_private.param<int>("stm32_baudrate",stm32_baudrate_,115200);
	
	
	assert(!obd_can_port_name_.empty() && !stm32_port_name_.empty());
	assert(max_steering_speed_>0);
	
	cmd1_sub = nh.subscribe("/controlCmd1",1,&BaseControl::callBack1,this);
	cmd2_sub = nh.subscribe("/controlCmd2",1,&BaseControl::callBack2,this);
	
	state1_pub = nh.advertise<little_ant_msgs::State1>("vehicleState1",10);
	state2_pub = nh.advertise<little_ant_msgs::State2>("vehicleState2",10);
	state3_pub = nh.advertise<little_ant_msgs::State3>("vehicleState3",10);
	state4_pub = nh.advertise<little_ant_msgs::State4>("vehicleState4",10);
	std_msg_pub = nh.advertise<std_msgs::UInt64>("vehicle_info",1);
	
	timer_ = nh.createTimer(ros::Duration(0.03), &BaseControl::timer_callBack, this);
	
	if(!openSerial(stm32_serial_port_,stm32_port_name_,stm32_baudrate_))
		return false;
	
	if(!can2serial.configure_port(obd_can_port_name_.c_str()))
	{
		ROS_INFO("open port %s failed",obd_can_port_name_.c_str());
		return false;
	}
	else
		ROS_INFO("open port %s successfully",obd_can_port_name_.c_str());
	/*
	can2serial.clearCanFilter();
	
	can2serial.setCanFilter_alone(0x01,ID_STATE1); usleep(10000);
	can2serial.setCanFilter_alone(0x02,ID_STATE2); usleep(10000);
	can2serial.setCanFilter_alone(0x03,ID_STATE3); usleep(10000);
	can2serial.setCanFilter_alone(0x04,ID_STATE4); usleep(10000);
	
	can2serial.configBaudrate(500);
	*/
	can2serial.StartReading();
		
	ROS_INFO("System initialization completed");
	
	//usleep(1500000);
	
	//can2serial.clearCanFilter();
	return true;
}

void BaseControl::run()
{
	readFromStm32_thread_ptr_ = boost::shared_ptr<boost::thread > 
		(new boost::thread(boost::bind(&BaseControl::read_stm32_port, this)));
		
	boost::thread parse_msg(boost::bind(&BaseControl::parse_obdCanMsg,this)); 
	
	ros::spin();
}

void BaseControl::parse_obdCanMsg()
{
	CanMsg_t canMsg;

	while(ros::ok())
	{
		//ROS_INFO("parse_obdCanMsg  ing.....");
		usleep(3000);
		if(!can2serial.getCanMsg(canMsg))
		{
			//ROS_INFO("nothing....");
			continue;
		}
			
		//ROS_INFO("ID:%x",canMsg.ID);
			
		switch(canMsg.ID)
		{
			case ID_STATE1:
				state1.act_gear = canMsg.data[0] >>4;
				state1.accel_pedal_position = canMsg.data[1] * 0.4;
				state1.brake_pedal = canMsg.data[2] & 0x01;
				state1.accel_pedal_position_valid = !(canMsg.data[2] & 0x02);
				state1.brake_pedal_valid = !(canMsg.data[2] & 0x04);
				state1.act_gear_valid = !(canMsg.data[2]&0x10);
				state1.vehicle_ready = bool(canMsg.data[2]&0x20);
				state1.driverless_mode = bool(canMsg.data[2]&0x40);
				state1.header.stamp = ros::Time::now();
				state1_pub.publish(state1);
				break;
				
			case ID_STATE2:
				state2.wheel_speed_FL_valid = !(canMsg.data[0] >>6);
				state2.wheel_speed_FL = ((canMsg.data[0]&0x3f)*256+canMsg.data[1])*0.0625;
				state2.wheel_speed_FR_valid = !(canMsg.data[1] >>6);
				state2.wheel_speed_FR = ((canMsg.data[2]&0x3f)*256+canMsg.data[3])*0.0625;
				
				state2.wheel_speed_RL_valid = !(canMsg.data[4] >>6);
				state2.wheel_speed_RL = ((canMsg.data[4]&0x3f)*256+canMsg.data[5])*0.0625;
				
				state2.wheel_speed_RR_valid = !(canMsg.data[6] >>6);
				state2.wheel_speed_RR = ((canMsg.data[6]&0x3f)*256+canMsg.data[7])*0.0625;
				
				{
				size_t i =0;
				float speed = 0.0;
				if(state2.wheel_speed_FL_valid==true) {i++; speed += state2.wheel_speed_FL;}
				if(state2.wheel_speed_FR_valid==true) {i++; speed += state2.wheel_speed_FR;}
				if(state2.wheel_speed_RL_valid==true) {i++; speed += state2.wheel_speed_RL;}
				if(state2.wheel_speed_RR_valid==true) {i++; speed += state2.wheel_speed_RR;}
				
				state2.vehicle_speed = speed/i /3.6; //m/s
				}
				state2.header.stamp = ros::Time::now();
				state2_pub.publish(state2);
				break;
				
			case ID_STATE3:
				state3.driverless_mode= bool(canMsg.data[0]&0x01);
				state3.turn_light_R = bool(canMsg.data[1]&0x01);
				state3.turn_light_L = bool(canMsg.data[1]&0x02);
				
				state3.parkTail_light = bool(canMsg.data[1]&0x08);
				state3.high_beam = bool(canMsg.data[1]&0x10);
				state3.low_beam = bool(canMsg.data[1]&0x20);
				state3.brake_light = bool(canMsg.data[2]&0x01);
				state3.horn = bool(canMsg.data[2]&0x02);
				state3.header.stamp = ros::Time::now();
				state3_pub.publish(state3);
				break;
				
			case ID_STATE4:
				state4.driverless_mode = bool(canMsg.data[0]&0x01);
				state4.steeringAngle = 1080.0-(canMsg.data[1]*256+canMsg.data[2])*0.1;
				//std::cout << state4.steeringAngle << std::endl;
				state4.roadwheelAngle = state4.steeringAngle/g_steering_gearRatio;
				if(state4.steeringAngle==6553.5)
					state4.steeringAngle_valid = 0;
				else
					state4.steeringAngle_valid = 1;
				state4.steeringAngle_speed = canMsg.data[3]*4; // deg/s
				
				state4.header.stamp = ros::Time::now();
				state4_pub.publish(state4);
				break;
				
			default:
				break;
		}
	}
}

void BaseControl::read_stm32_port()
{
	size_t len;
	
	stm32_serial_port_->flushInput();
	
	while(ros::ok())
	{
		//ROS_INFO("read_stm32_port  ing.....");
		usleep(5000);
		try 
		{
			len = stm32_serial_port_->read(stm32_data_buf, STM32_MAX_NOUT_SIZE);
			//ROS_INFO("read_stm32_port get %d bytes",len);
		} 
		catch (std::exception &e) 
		{
	        std::stringstream output;
	        output << "Error reading from serial port: " << e.what();
	        std::cout << output.str() <<std::endl;
    	}
    	if(len == 0) continue;
    	
    	 //for(int i=0;i<len;i++)
    	 //	printf("%x\t",stm32_data_buf[i]);
    	 //std::cout << std::endl;
    	
		Stm32BufferIncomingData(stm32_data_buf, len);
	}
}

void BaseControl::Stm32BufferIncomingData(unsigned char *message, unsigned int length)
{
	static int buffer_index = 0;
	static int bytes_remaining =0;
	
	// add incoming data to buffer
	for (unsigned int ii=0; ii<length; ii++) 
	{// make sure bufIndex is not larger than buffer
		if(buffer_index >= STM32_MAX_PKG_BUF_LEN)
		{
			buffer_index = 0;
			  printf("Overflowed receive buffer. Buffer cleared.");
		}
		switch(buffer_index)
		{
			case 0: //nothing
				if(message[ii]==Stm32MsgHeaderByte0)
				{
					stm32_pkg_buf[buffer_index++] = message[ii];
				}
				bytes_remaining = 0;
				break;
			case 1:
				if(message[ii]==Stm32MsgHeaderByte1)
				{
					stm32_pkg_buf[buffer_index++] = message[ii];
					bytes_remaining =2; //2 bytes pkgLen
				}
				else
				{
					buffer_index = 0;
					bytes_remaining = 0;
				}
				break;
			case 2:
			case 3:
				stm32_pkg_buf[buffer_index++]=message[ii];
				bytes_remaining --;
				if(bytes_remaining == 0)
				{
					bytes_remaining = (stm32_pkg_buf[2] << 8) + stm32_pkg_buf[3] ;
					
					//根据实际发送的包长最大小值进行限定(多重数据正确保障) 
					if(bytes_remaining > 9 || bytes_remaining < 2)  
					{
						buffer_index = 0;
						break;
					}
				}
				break;
			default:
				stm32_pkg_buf[buffer_index++] = message[ii];
				bytes_remaining --;
				if(bytes_remaining == 0)
				{
					parse_stm32_msgs();
					buffer_index = 0;
				}
				break;
		}
	}// end for
}

void BaseControl::parse_stm32_msgs()
{
	if(stm32_msg1Ptr_->checkNum != generateCheckNum(stm32_msg1Ptr_,ntohs(stm32_msg1Ptr_->pkgLen)+4))
		return ;

	if(stm32_msg1Ptr_->id == 0x01)
	{
		//mutex_.lock();
		if(stm32_msg1Ptr_->is_start && !stm32_msg1Ptr_->is_emergency_brake && !is_driverlessMode_)
		{
			this->setDriverlessMode();//启动无人驾驶模式
			ROS_INFO("setDriverlessMode ok ^-^");
			stm32_serial_port_->flushInput();
			this->is_driverlessMode_ = true;
		}
		else if(!stm32_msg1Ptr_->is_start && is_driverlessMode_)
		{
			this->is_driverlessMode_ = false;
			this->exitDriverlessMode();
			ROS_INFO("exitDriverlessMode ok ^-^");
		}
			
		//printf("is_start:%d\t is_emergency_brake:%d\n",stm32_msg1_.is_start,stm32_msg1_.is_emergency_brake);
		//mutex_.unlock();
	}
}

//先发送启动无人驾驶模式指令，一段时间之后再发送换挡指令
void BaseControl::setDriverlessMode()
{
	ROS_INFO("set driverless mode ing ............");
	
	*(unsigned long int*)canMsg_cmd1.data = 0;
	*(unsigned long int*)canMsg_cmd2.data = 0;

	canMsg_cmd1.data[0] = 0x01; //driverless_mode
	//send driverless mode cmd
	for(int count=0; ros::ok(); )
	{
		can2serial.sendCanMsg(canMsg_cmd1);
		usleep(20000);
	//when setting the vehicle into driverless mode
	//the steering will be back to the middle automaticly, and the steer rotate speed is suitable
	//but if we immdiately send the steering cmd, the steer will rotate very fast, even cause EPS to fail
	//so we should wait a minute before we send steering cmd
	//if we just use sleep(x) but not keep cmd being sending ,the driverless system in vehicle may exit
		if(state4.driverless_mode)
		{
			++count;
			if(count == 5)
				break;
		}
	}
	
	const uint16_t middle_steeringValue = 10800; //middle
	canMsg_cmd2.data[4] =  uint8_t(middle_steeringValue / 256);
	canMsg_cmd2.data[5] = uint8_t(middle_steeringValue % 256);
	
	for(size_t count = 0; ros::ok() && state1.act_gear != 1; ++count)
	{
		if(count%6==0)
			canMsg_cmd2.data[0] = 0x00;
		else
			canMsg_cmd2.data[0] = 0x01;
			
		can2serial.sendCanMsg(canMsg_cmd2);
		usleep(10000);
	}
	canMsg_cmd2.data[0] = 0x01;
	
	//to Ensure steering stability at set value
	for(size_t count=0; count<50; count++)
	{
		can2serial.sendCanMsg(canMsg_cmd2);
		usleep(10000);
		can2serial.sendCanMsg(canMsg_cmd2);
		usleep(10000);
		can2serial.sendCanMsg(canMsg_cmd1);
	}
	
	stm32_serial_port_->flushInput();
}
void BaseControl::exitDriverlessMode()
{
	ROS_INFO("driverless mode exited ............");
	*(unsigned long int*)canMsg_cmd1.data = 0;
	*(unsigned long int*)canMsg_cmd2.data = 0;

	canMsg_cmd1.data[0] = 0x00; //driverless_mode
	canMsg_cmd2.data[0] = 0x00; //set_gear 0
	
	uint16_t steeringAngle = 10800; //middle
	
	canMsg_cmd2.data[4] =  uint8_t(steeringAngle / 256);
	canMsg_cmd2.data[5] = uint8_t(steeringAngle % 256);
	
	for(int i=0;i<5;i++)
	{
		usleep(10000);
		can2serial.sendCanMsg(canMsg_cmd1);
		usleep(20000);
		can2serial.sendCanMsg(canMsg_cmd2);
	}
	stm32_serial_port_->flushInput();
}

void BaseControl::timer_callBack(const ros::TimerEvent& event)
{
	//send cmd to stm32
	send_to_stm32_buf[5] = stm32_brake_ & 0x7f;
	if(state4.driverless_mode)
		send_to_stm32_buf[5] |= 0x80;

	send_to_stm32_buf[7] = generateCheckNum(send_to_stm32_buf,8);
	stm32_serial_port_->write(send_to_stm32_buf,8);
	
	//publish other msg
	static StateUnion_t msg;
	msg.state.act_gear = state1.act_gear;
	msg.state.driverless_mode = state4.driverless_mode;
	msg.state.hand_brake = 0;
	msg.state.emergency_brake = 0;
	msg.state.car_state = state1.vehicle_ready;
	msg.state.speed = state2.vehicle_speed * 3.6 *100;
	msg.state.roadwheelAngle = state4.roadwheelAngle *100 + 5000;
	static std_msgs::UInt64 ros_msg;
	ros_msg.data = msg.data;
	std_msg_pub.publish(ros_msg);

}

void BaseControl::callBack1(const little_ant_msgs::ControlCmd1::ConstPtr msg)
{
	if(!is_driverlessMode_)
		return ;
		
	if(msg->set_driverlessMode)
		canMsg_cmd1.data[0] |= 0x01;
	else
		canMsg_cmd1.data[0] &= 0xfe;
	
	if(msg->set_remoteStart)
		canMsg_cmd1.data[0] |= 0x02;
	else
		canMsg_cmd1.data[0] &= 0xfd;
		
	if(msg->set_handBrake)
		canMsg_cmd1.data[0] |= 0x04;
	else
		canMsg_cmd1.data[0] &= 0xfb;
	
	if(msg->set_turnLight_R)
		canMsg_cmd1.data[1] |= 0x01;
	else
		canMsg_cmd1.data[1] &= 0xfe;
		
	if(msg->set_turnLight_L)
		canMsg_cmd1.data[1] |= 0x02;
	else
		canMsg_cmd1.data[1] &= 0xfd;
		
	if(msg->set_lowBeam)
		canMsg_cmd1.data[1] |= 0x20;
	else
		canMsg_cmd1.data[1] &= 0xdf;
		
	if(msg->set_reverseLight)
		canMsg_cmd1.data[1] |= 0x40;
	else
		canMsg_cmd1.data[1] &= 0xbf;
	
	if(msg->set_brakeLight)
		canMsg_cmd1.data[2] |= 0x01;
	else
		canMsg_cmd1.data[2] &= 0xfe;
		
	if(msg->set_horn)
		canMsg_cmd1.data[2] |= 0x02;
	else
		canMsg_cmd1.data[2] &= 0xfd;
	
	can2serial.sendCanMsg(canMsg_cmd1);
}


void BaseControl::callBack2(const little_ant_msgs::ControlCmd2::ConstPtr msg)
{
	if(!is_driverlessMode_)
		return ;
		
	float set_speed = msg->set_speed;

	float set_brake = msg->set_brake;
	
	int currentSpeed = state2.vehicle_speed * 3.6;
	
	//当设定速度低于当前速度时，制动
	if(currentSpeed  > 2.0 + msg->set_speed)
	{
		set_brake = (currentSpeed - msg->set_speed - 2.0) *3 + 40;
	}
	
	set_brake = (set_brake > msg->set_brake) ? set_brake :msg->set_brake;
	
	if(set_brake >0.0)
		set_speed = 0.0;
	else if(set_speed > MAX_SPEED-1) 
		set_speed = MAX_SPEED-1;
	//increment越大，加速度越大
	//设定速度越低，加速越快
	float increment = 3.0/(currentSpeed/5+1);
	
	if(set_speed - currentSpeed > increment )
		set_speed = currentSpeed + increment;
			
		
	canMsg_cmd2.data[0] &= 0xf0; //clear least 4bits
	canMsg_cmd2.data[0] |= (msg->set_gear)&0x0f;
	
	canMsg_cmd2.data[1] = uint8_t(set_speed * 10 * 15.0 / MAX_SPEED);
	
	if(set_brake>40)
		canMsg_cmd2.data[2] = uint8_t(40 *2.5);
	else
		canMsg_cmd2.data[2] = uint8_t(set_brake *2.5);
	
	canMsg_cmd2.data[3] = uint8_t(msg->set_accelerate *50);
	
	static float last_set_steeringAngle = state4.steeringAngle;
	float current_set_steeringAngle = msg->set_roadWheelAngle * g_steering_gearRatio;  // -540~540deg
	
	if(current_set_steeringAngle>480.0) current_set_steeringAngle=480.0;
	else if(current_set_steeringAngle<-480.0) current_set_steeringAngle =-480.0;
	
	if(current_set_steeringAngle - last_set_steeringAngle > max_steering_speed_)
		current_set_steeringAngle = last_set_steeringAngle + max_steering_speed_;
	else if(current_set_steeringAngle - last_set_steeringAngle < -max_steering_speed_)
		current_set_steeringAngle = last_set_steeringAngle - max_steering_speed_;
		
	last_set_steeringAngle = current_set_steeringAngle;
	
	uint16_t steeringAngle = 10800 - (current_set_steeringAngle*10 - 20) ; //steering offset
	
	canMsg_cmd2.data[4] =  uint8_t(steeringAngle / 256);
	canMsg_cmd2.data[5] = uint8_t(steeringAngle % 256);
	
	if(msg->set_emergencyBrake)
		canMsg_cmd2.data[6] |= 0x10;
	else
		canMsg_cmd2.data[6] &= 0xef;
		
	can2serial.sendCanMsg(canMsg_cmd2);
	
	if(set_brake > 100)
		stm32_brake_ = 100;
	else if(set_brake >40)
		stm32_brake_ = (set_brake - 40)/60.0 * 100;
	else
		stm32_brake_ = 0;
	
//	std::cout << "stm32_brake_: " << int(stm32_brake_) << std::endl;
}

uint8_t BaseControl::generateCheckNum(const void* voidPtr,size_t len)
{
	const uint8_t *ptr = (const uint8_t *)voidPtr;
    uint8_t sum=0;

    for(int i=2; i<len-1 ; i++)
    {
        sum += ptr[i];
    }
    return sum;
}

int main(int argc,char**argv)
{
	BaseControl base_control;
	
	if(base_control.init(argc,argv))
		base_control.run();
	
	printf("base_control_node has exited");
	return 0;
}












