#include<msgHandler.h>

MsgHandler::MsgHandler()
{
	canMsg_cmd1.ID = ID_CMD_1;
    canMsg_cmd1.len = 8;
    *(long *)canMsg_cmd1.data = 0;

    canMsg_cmd2.ID = ID_CMD_2;
    canMsg_cmd2.len = 8;
    *(long *)canMsg_cmd2.data = 0;
    canMsg_cmd2.data[4] = 0xFF;
    canMsg_cmd2.data[5] = 0xFF; //set the steer angle value invalid
}

MsgHandler::~MsgHandler()
{
	
}

bool MsgHandler::init(int argc,char**argv)
{
	ros::init(argc,argv,"base_control");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("port_name", port_name_, "/dev/ttyUSB0");
	
	cmd1_sub = nh.subscribe("/controlCmd1",10,&MsgHandler::callBack1,this);
	cmd2_sub = nh.subscribe("/controlCmd2",10,&MsgHandler::callBack2,this);
	
	state1_pub = nh.advertise<little_ant_msgs::State1>("vehicleState1",10);
	state2_pub = nh.advertise<little_ant_msgs::State2>("vehicleState2",10);
	state3_pub = nh.advertise<little_ant_msgs::State3>("vehicleState3",10);
	state4_pub = nh.advertise<little_ant_msgs::State4>("vehicleState4",10);
	
	if(!can2serial.openPort(port_name_.c_str()))
	{
		ROS_INFO("open port %s failed",port_name_.c_str());
		return 0;
	}
	
	can2serial.setCanFilter(0x01,ID_STATE1,ID_STATE1); usleep(10000);
	can2serial.setCanFilter(0x02,ID_STATE2,ID_STATE2); usleep(10000);
	can2serial.setCanFilter(0x03,ID_STATE3,ID_STATE3); usleep(10000);
	can2serial.setCanFilter(0x04,ID_STATE4,ID_STATE4); usleep(10000);
	
	can2serial.run();
	
	while(!can2serial.configBaudrate(500))
	{
		ROS_INFO("set baudrate...");
		usleep(10000);
	}
		
	ROS_INFO("System initialization completed");
	
	//usleep(1500000);
	
	//can2serial.clearCanFilter();
	return 1;
}

void MsgHandler::run()
{
	boost::thread parse_msg(boost::bind(&MsgHandler::parse,this)); 
	
	ros::spin();
}

void MsgHandler::parse()
{
	STD_CAN_MSG canMsg;

	while(ros::ok())
	{
		//ROS_INFO("ing.....");
		usleep(3000);
		if(!can2serial.getMsg(canMsg))
		{
			//ROS_INFO("nothing....");
			continue;
		}
			
		ROS_INFO("ID:%x",canMsg.ID);
			
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
				
				state3_pub.publish(state3);
				break;
				
			case ID_STATE4:
				state4.driverless_mode = bool(canMsg.data[0]&0x01);
				state4.steeringAngle = 1080.0-(canMsg.data[1]*256+canMsg.data[2])*0.1;
				if(state4.steeringAngle==6553.5)
					state4.steeringAngle_valid = 0;
				else
					state4.steeringAngle_valid = 1;
				state4.steeringAngle_speed = canMsg.data[3]*4; // deg/s
				state4_pub.publish(state4);
				break;
				
			default:
				break;
				
		}
	}
}



void MsgHandler::callBack1(const little_ant_msgs::ControlCmd1::ConstPtr msg)
{
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
	
	can2serial.sendStdCanMsg(canMsg_cmd1);
	
}

void MsgHandler::callBack2(const little_ant_msgs::ControlCmd2::ConstPtr msg)
{
	canMsg_cmd2.data[0] &= 0xf0;
	canMsg_cmd2.data[0] |= (msg->set_gear)&0x0f;
	
	canMsg_cmd2.data[1] = uint8_t(msg->set_speed * 10);
	
	canMsg_cmd2.data[2] = uint8_t(msg->set_brake *2.5);
	
	canMsg_cmd2.data[3] = uint8_t(msg->set_accelerate *50);
	
	uint16_t steeringAngle = 10800 - msg->set_steeringAngle*10;
	
	canMsg_cmd2.data[4] =  uint8_t(steeringAngle / 256);
	canMsg_cmd2.data[5] = uint8_t(steeringAngle % 256);
	
	if(msg->set_emergencyBrake)
		canMsg_cmd2.data[6] |= 0x10;
	else
		canMsg_cmd2.data[6] &= 0xef;
		
	can2serial.sendStdCanMsg(canMsg_cmd2);
	
	
		
}
