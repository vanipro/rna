#include <ros/ros.h>
#include <angles/angles.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <control/DriverCmd.h>
#include <geometry_msgs/Twist.h>

#include <serial/serial.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <mutex>
#include <queue>
#include <string>
#include <algorithm>



class CobraHardware : public hardware_interface::RobotHW 
{
public:
	CobraHardware(ros::NodeHandle& nh_, const std::string& left_front_wheel,  const std::string& left_rear_wheel,
                                        const std::string& right_front_wheel, const std::string& right_rear_wheel);

private:
	std::shared_ptr<controller_manager::ControllerManager> controller_manager_; 
    ros::Timer    timer_;
    ros::Duration elapsed_time_;
    
    ros::Subscriber cmd_vel_sub_;
	ros::ServiceServer driver_srv_;

	serial::Serial ser_;
    std::string port_        = "/dev/ttyACM1";
    int         baud_ 		 = 115200;
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(1000);
    
    hardware_interface::JointStateInterface joint_state_interface_;
	hardware_interface::VelocityJointInterface velocity_joint_interface_;

    double wheel_base_   = 0.153,
		   wheel_radius_ = 0.0745; 
	
	double joint_cmd_[4],
    	   joint_effort_[4],
    	   joint_velocity_[4],
    	   joint_position_[4];

    int loop_hz_ = 30;
	int prev_odl_ = 0.0, 
		prev_odr_ = 0.0;

	std::mutex cmd_mutex_;
	std::queue<nlohmann::json> cmd_queue_;

    void init(ros::NodeHandle& nh_, const std::string& left_front_wheel, const std::string& left_rear_wheel,
                                               const std::string& right_front_wheel, const std::string& right_rear_wheel);
    void tick(const ros::TimerEvent& e);
	void recv();
	void send();
	bool conn();
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
	bool driverCallback(control::DriverCmd::Request& req, control::DriverCmd::Response& res);

	static inline int radps2rpm(double rad_per_sec) 
	{
        const double rpm = rad_per_sec * 60.0 / (2.0 * M_PI);
        int tenth = static_cast<int>(std::round(rpm * 10.0));

        return std::min(std::max(tenth, -1800), 1800);
    }
};

CobraHardware::CobraHardware(ros::NodeHandle& nh_, const std::string& left_front_wheel,  const std::string& left_rear_wheel,
                                                   const std::string& right_front_wheel, const std::string& right_rear_wheel)
{
	nh_.param<std::string>("port",         port_,         port_);
    nh_.param<int>        ("baud",         baud_,         baud_);
    nh_.param<int>        ("loop_hz",      loop_hz_,      loop_hz_);
    nh_.param<double>     ("wheel_base",   wheel_base_,   wheel_base_);
    nh_.param<double>     ("wheel_radius", wheel_radius_, wheel_radius_);


	if (!conn()) 
        ROS_WARN("Failed to open serial (port=%s, baud=%d). Running in simulation mode.", port_.c_str(), baud_);
	
	init(nh_, left_front_wheel, left_rear_wheel, right_front_wheel, right_rear_wheel);

	cmd_queue_.push((nlohmann::json){{"T", 600}});
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1, loop_hz_)), &CobraHardware::tick, this);
	driver_srv_ = nh_.advertiseService("driver", &CobraHardware::driverCallback, this);

    //cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &CobraHardware::cmdVelCallback, this);
    
    ROS_INFO("CobraHardware node started at %s @ %d, loop %.1d Hz", port_.c_str(), baud_, loop_hz_);
    ROS_INFO("Controller manager initialized and ready for controllers");
    ROS_INFO("Subscribed to /cmd_vel for direct control");
}


void CobraHardware::init(ros::NodeHandle& nh_, const std::string& left_front_wheel,  const std::string& left_rear_wheel,
                                               const std::string& right_front_wheel, const std::string& right_rear_wheel) 
{
	(void)nh_;
	std::string names[4] = { left_front_wheel, left_rear_wheel, right_front_wheel, right_rear_wheel };

	for(int i = 0; i < 4; ++i) 
	{
		hardware_interface::JointStateHandle jsh(names[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		joint_state_interface_.registerHandle(jsh);
	}

    for(int i = 0; i < 4; ++i)
    {
        hardware_interface::JointHandle handle(joint_state_interface_.getHandle(names[i]),  &joint_cmd_[i]);
        velocity_joint_interface_.registerHandle(handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    
    ROS_INFO("Hardware interfaces registered successfully");
}

void CobraHardware::tick(const ros::TimerEvent& e) 
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    recv();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    send();
    
}

void CobraHardware::recv() {
	if (!ser_.isOpen()) 
	{
		static double time = 0.0;
		time += elapsed_time_.toSec();
		return;
	}

	nlohmann::json in;
	try
	{
		serial::Timeout short_timeout = serial::Timeout::simpleTimeout(10);
		ser_.setTimeout(short_timeout);
		
		std::string line = ser_.readline(4096, "\n");
		const std::size_t end = line.find_last_not_of("\r\n");

		if (end == std::string::npos) line.clear();   
		else 						  line.resize(end + 1);

		in = nlohmann::json::parse(line, nullptr, false);
        if (in.is_discarded()) 
            return;

		if (in.contains("odl") && in.contains("odr")) {
			const double odl = in.value("odl", 0.0) * 0.01;
			const double odr = in.value("odr", 0.0) * 0.01;
			
			const double dtheta_l = odl / wheel_radius_;
			const double dtheta_r = odr / wheel_radius_;
			
			prev_odl_ = odl;
			prev_odr_ = odr;
			
			joint_position_[0] = dtheta_l;
			joint_position_[1] = dtheta_l;
			joint_position_[2] = dtheta_r;
			joint_position_[3] = dtheta_r;

			//ROS_INFO(">> Received: %s", line.c_str());
		}
	} 
	catch (const std::exception& e) 
	{
        ROS_WARN_THROTTLE(1.0, "recv() exception: %s", e.what());
        return;
	}
}

void CobraHardware::send() {
	if (!ser_.isOpen()) 
		return;

	try 
	{
		std::lock_guard<std::mutex> lock(cmd_mutex_);

		nlohmann::json cmd_json; 
		if (!cmd_queue_.empty()) 
		{
			cmd_json = cmd_queue_.front();
            cmd_queue_.pop();
		}
		else cmd_json = {
			{"T", 1}, 
			{"L", radps2rpm(joint_cmd_[0])},
			{"R", radps2rpm(joint_cmd_[2])},
		};

        std::string payload = cmd_json.dump() + "\n";

        ser_.write(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
        ser_.flush();
        
        ROS_INFO("<< Sent: %s", payload.substr(0, payload.length()-1).c_str());
        
    } 
	catch (const std::exception& e) 
	{
        ROS_WARN_THROTTLE(1.0, "send() exception: %s", e.what());
		return;
    }
}

bool CobraHardware::conn() {
    try 
	{
        ser_.setPort(port_);
        ser_.setBaudrate(baud_);
        ser_.setTimeout(timeout_);

        ser_.open();
        if (!ser_.isOpen())
			return false;

        ros::Duration(1.0).sleep();
        
        ser_.flushInput();
        ser_.flushOutput();
        
        if (ser_.available() > 0)
            ser_.read(ser_.available());
        
        serial::Timeout read_timeout = serial::Timeout::simpleTimeout(100);
        ser_.setTimeout(read_timeout);
        
        ROS_INFO("Serial opened successfully on %s @ %d baud", port_.c_str(), baud_);
        return true;
    } 
	catch (const std::exception& e) 
	{
        ROS_ERROR("Serial connection failed: %s", e.what());
        return false;
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "cobra_hardware");
    ros::NodeHandle nh;

    try 
	{
		CobraHardware cobra_hardware(nh, "left_front_wheel_joint", "left_rear_wheel_joint",
                                         "right_front_wheel_joint", "right_rear_wheel_joint");		
        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
    }
	catch (const std::exception& e) 
	{
        ROS_FATAL("Exiting: %s", e.what());
        return 1;
    }

    return 0;
}

void CobraHardware::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;
    
    double left_vel = (linear_vel - angular_vel * wheel_base_ / 2.0) / wheel_radius_;
    double right_vel = (linear_vel + angular_vel * wheel_base_ / 2.0) / wheel_radius_;
    
    joint_cmd_[0] = left_vel;
    joint_cmd_[1] = right_vel;
    
    ROS_INFO("Received cmd_vel: linear=%.3f, angular=%.3f -> L=%.3f, R=%.3f", 
             linear_vel, angular_vel, left_vel, right_vel);
}

bool CobraHardware::driverCallback(control::DriverCmd::Request& req, 
								   control::DriverCmd::Response& res) 
{
	try 
	{
		nlohmann::json cmd = {{"T", req.T}};

		for (const auto& param : req.params) 
		{
			size_t pos = param.find(':');
			if (pos != std::string::npos)
				cmd[param.substr(0, pos)] = std::stoi(param.substr(pos + 1));
		}
        
        std::lock_guard<std::mutex> lock(cmd_mutex_);
		cmd_queue_.push(cmd);

		res.success = true;
		res.message = "ok";
		ROS_INFO("Queued: %s", cmd.dump().c_str());
		return true;

	} 
	catch (const std::exception& e) 
	{
		res.success = false;
		res.message = std::string("JSON parse error: ") + e.what();
		ROS_ERROR("Failed to parse DriverCmd: %s", e.what());
		return false;
	}
}
