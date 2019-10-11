#include <path_tracking/State.h>
#include <ros/ros.h>
#include <unistd.h>
#include <fstream>
#include <cmath>

class Recorder
{
public:
	Recorder(){};
	~Recorder()
	{
		if(out_file_.is_open())
			out_file_.close();
	}
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle private_nh("~");
	
		private_nh.param<std::string>("file_path",file_path_,"");
		private_nh.param<std::string>("file_name",file_name_,"");
	
		if(file_path_.empty() || file_name_.empty())
		{
			ROS_ERROR("please input file path and file name in launch file!!!");
			return false;
		}
		std::string full_file_name = file_path_ + file_name_;
		out_file_.open(full_file_name);
		if(!out_file_.is_open())
		{
			ROS_ERROR("open %s failed!!!", full_file_name.c_str());
			return false;
		}
		
		std::string state_topic = private_nh.param<std::string>("state_topic", "");
		sub_state_ = nh.subscribe(state_topic, 1, &Recorder::callback, this);
		return true;
	}
	void callback(const path_tracking::State::ConstPtr& state)
	{
		out_file_ << std::fixed << std::setprecision(3) << state->header.stamp.toSec() << "\t"
				  << state->position_x << "\t" << state->position_y << "\t" << state->yaw << "\t"
				  << state->vehicle_speed << "\t" << state->roadwheel_angle << "\t"
				  << state->lateral_error << "\t" << state->yaw_error << "\r\n";
	}
	
	
private:
	std::string file_path_;
	std::string	file_name_;
	std::ofstream out_file_;
	
	ros::Subscriber sub_state_;
};

int main(int argc,char**argv)
{
	ros::init(argc,argv,"record_pathtracking_state_node");
	Recorder recorder;
	if(!recorder.init())
		return 1;

	ros::spin();
	
	return 0;
}


