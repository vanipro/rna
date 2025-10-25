#include <ros/ros.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <angles/angles.h>
#include <serial/serial.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <algorithm>

class CobraHardware : public hardware_interface::RobotHW {
public:
    CobraHardware(ros::NodeHandle &nh);

private:
    controller_manager::ControllerManager controller_manager_;
    int loop_hz_ = 100;
    ros::Timer timer_;
    ros::Duration elapsed_;

    bool sim_mode_ = false;
    bool use_ros_control_ = true;
    std::string feedback_type_ = "none";

    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster tf_br_;

    serial::Serial ser_;
    std::string port_ = "/dev/ttyACM1";
    int baud_ = 115200;
    serial::Timeout timeout_ = serial::Timeout::simpleTimeout(1000);

    hardware_interface::JointStateInterface jnt_state_if_;
    hardware_interface::VelocityJointInterface jnt_vel_if_;

    double wheel_base_ = 0.153;
    double wheel_radius_ = 0.0745;

    std::string odom_frame_ = "odom";
    std::string base_frame_ = "base_link";

    std::vector<std::string> left_wheels_;
    std::vector<std::string> right_wheels_;

    std::vector<double> jnt_cmd_;
    std::vector<double> jnt_eff_;
    std::vector<double> jnt_vel_;
    std::vector<double> jnt_pos_;

    double x_ = 0.0;
    double y_ = 0.0;
    double th_ = 0.0;
    
    int prev_odl_ = 0;
    int prev_odr_ = 0;

    void initInterfaces_();
    void onTick_(const ros::TimerEvent &e);
    void read_();
    void write_();
    bool openSerial_();
    void cmdVelCb_(const geometry_msgs::Twist::ConstPtr &msg);
    void integrateOdom_(double v_l, double v_r, double dt);

    static inline int radpsToRpm10_(double v) {
        int t = (int)std::round((v * 60.0 / (2.0 * M_PI)) * 10.0);
        return std::max(-1800, std::min(1800, t));
    }
};

CobraHardware::CobraHardware(ros::NodeHandle &nh) : controller_manager_(this, nh) {
    nh.param<std::string>("port", port_, port_);
    nh.param<int>("baud", baud_, baud_);
    nh.param<int>("loop_hz", loop_hz_, loop_hz_);
    nh.param<double>("wheel_base", wheel_base_, wheel_base_);
    nh.param<double>("wheel_radius", wheel_radius_, wheel_radius_);
    nh.param<bool>("sim_mode", sim_mode_, sim_mode_);
    nh.param<bool>("use_ros_control", use_ros_control_, use_ros_control_);
    nh.param<std::string>("feedback_type", feedback_type_, feedback_type_);
    nh.param<std::string>("odom_frame", odom_frame_, odom_frame_);
    nh.param<std::string>("base_frame", base_frame_, base_frame_);

    left_wheels_  = {"front_left_wheel", "rear_left_wheel"};
    right_wheels_ = {"front_right_wheel", "rear_right_wheel"};

    size_t n = left_wheels_.size() + right_wheels_.size();

    jnt_cmd_.assign(n, 0.0);
    jnt_eff_.assign(n, 0.0);
    jnt_vel_.assign(n, 0.0);
    jnt_pos_.assign(n, 0.0);

    if (!sim_mode_ && !openSerial_()) sim_mode_ = true;

    initInterfaces_();

    loop_hz_ = std::max(10, loop_hz_);
    timer_ = nh.createTimer(ros::Duration(1.0 / loop_hz_), &CobraHardware::onTick_, this);

    if (!use_ros_control_)
        cmd_vel_sub_ = nh.subscribe("/cmd_vel", 1, &CobraHardware::cmdVelCb_, this);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);
}

void CobraHardware::initInterfaces_() {
    for (size_t i = 0; i < left_wheels_.size(); ++i) {
        hardware_interface::JointStateHandle s(left_wheels_[i], &jnt_pos_[i], &jnt_vel_[i], &jnt_eff_[i]);
        jnt_state_if_.registerHandle(s);

        hardware_interface::JointHandle v(jnt_state_if_.getHandle(left_wheels_[i]), &jnt_cmd_[i]);
        jnt_vel_if_.registerHandle(v);

		ROS_INFO("HW reg: velocity joint '%s'", left_wheels_[i].c_str());
    }

    for (size_t i = 0; i < right_wheels_.size(); ++i) {
        size_t idx = left_wheels_.size() + i;
        hardware_interface::JointStateHandle s(right_wheels_[i], &jnt_pos_[idx], &jnt_vel_[idx], &jnt_eff_[idx]);
        jnt_state_if_.registerHandle(s);

        hardware_interface::JointHandle v(jnt_state_if_.getHandle(right_wheels_[i]), &jnt_cmd_[idx]);
        jnt_vel_if_.registerHandle(v);

		ROS_INFO("HW reg: velocity joint '%s'", right_wheels_[i].c_str());
    }

    registerInterface(&jnt_state_if_);
    registerInterface(&jnt_vel_if_);
}

void CobraHardware::onTick_(const ros::TimerEvent &e) {
    elapsed_ = ros::Duration(e.current_real - e.last_real);
    double dt = std::max(1e-3, elapsed_.toSec());

    read_();
    controller_manager_.update(ros::Time::now(), elapsed_);
    write_();

    double sl = 0.0, sr = 0.0;

    for (size_t i = 0; i < left_wheels_.size(); ++i)
        sl += jnt_vel_[i];

    for (size_t i = 0; i < right_wheels_.size(); ++i)
        sr += jnt_vel_[left_wheels_.size() + i];

    double v_l = (left_wheels_.empty() ? 0.0 : sl / left_wheels_.size()) * wheel_radius_;
    double v_r = (right_wheels_.empty() ? 0.0 : sr / right_wheels_.size()) * wheel_radius_;

    integrateOdom_(v_l, v_r, dt);
}

void CobraHardware::read_() {
    if (!ser_.isOpen()) {
        // Simulation mode - generate fake data
        static double time = 0.0;
        time += elapsed_.toSec();
        
        for (size_t i = 0; i < jnt_cmd_.size(); ++i) {
            jnt_pos_[i] += jnt_cmd_[i] * elapsed_.toSec();
            jnt_vel_[i] = jnt_cmd_[i];
        }
        return;
    }

    nlohmann::json in;
    try {
        serial::Timeout short_timeout = serial::Timeout::simpleTimeout(10);
        ser_.setTimeout(short_timeout);
        
        std::string line = ser_.readline(4096, "\n");
        
        if (line.empty()) return;

        const std::size_t end = line.find_last_not_of("\r\n");
        if (end == std::string::npos) {
            line.clear();   
        } else {
            line.resize(end + 1);
        }

        if (line.empty()) return;

        if (line[0] != '{' || line.back() != '}') {
            return;
        }

        in = nlohmann::json::parse(line, nullptr, false);
        if (in.is_discarded()) {
            return;
        }

        if (in.contains("odl") && in.contains("odr")) {
            const double odl = in.value("odl", 0.0) * 0.01;
            const double odr = in.value("odr", 0.0) * 0.01;
            
            const double dtheta_l = (odl - prev_odl_) / wheel_radius_;
            const double dtheta_r = (odr - prev_odr_) / wheel_radius_;
            
            prev_odl_ = odl;
            prev_odr_ = odr;
            
            jnt_pos_[0] += dtheta_l;
            jnt_pos_[1] += dtheta_r;
            
            const double dt = std::max(1e-3, elapsed_.toSec());
            jnt_vel_[0] = dtheta_l / dt;
            jnt_vel_[1] = dtheta_r / dt;
        }
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(1.0, "read_() exception: %s", e.what());
    }
}

void CobraHardware::write_() {
    if (!ser_.isOpen()) {
        return;
    }

    try {
        nlohmann::json out = {
            {"T", 1}, 
            {"L", radpsToRpm10_(jnt_cmd_[0])},
            {"R", radpsToRpm10_(jnt_cmd_[1])}
        };

        std::string payload = out.dump();
        payload.push_back('\n');

        ser_.write(reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
        ser_.flush();
    } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(1.0, "write_() exception: %s", e.what());
    }
}

bool CobraHardware::openSerial_() {
    try {
        ser_.setPort(port_);
        ser_.setBaudrate(baud_);
        ser_.setTimeout(timeout_);

        ser_.open();
        if (!ser_.isOpen()) {
            return false;
        }

        ros::Duration(1.0).sleep();
        ser_.flushInput();
        ser_.flushOutput();
        
        if (ser_.available() > 0) {
            ser_.read(ser_.available());
        }
        
        serial::Timeout read_timeout = serial::Timeout::simpleTimeout(100);
        ser_.setTimeout(read_timeout);
        
        ROS_INFO("Serial opened successfully on %s @ %d baud", port_.c_str(), baud_);
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Serial connection failed: %s", e.what());
        return false;
    }
}

void CobraHardware::cmdVelCb_(const geometry_msgs::Twist::ConstPtr &msg) {
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;
    
    double left_vel = (linear_vel - angular_vel * wheel_base_ / 2.0) / wheel_radius_;
    double right_vel = (linear_vel + angular_vel * wheel_base_ / 2.0) / wheel_radius_;
    
    jnt_cmd_[0] = left_vel;
    jnt_cmd_[1] = right_vel;
}

void CobraHardware::integrateOdom_(double v_l, double v_r, double dt) {
    double v = (v_l + v_r) / 2.0;
    double w = (v_r - v_l) / wheel_base_;
    
    x_ += v * cos(th_) * dt;
    y_ += v * sin(th_) * dt;
    th_ += w * dt;
    
    // Publish odometry
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = sin(th_ / 2.0);
    odom.pose.pose.orientation.w = cos(th_ / 2.0);
    
    odom.twist.twist.linear.x = v;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = w;
    
    odom_pub_.publish(odom);
    
    // Publish transform
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = sin(th_ / 2.0);
    transform.transform.rotation.w = cos(th_ / 2.0);
    
    tf_br_.sendTransform(transform);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware");
    ros::NodeHandle nh("~");

    CobraHardware hw(nh);

    ros::MultiThreadedSpinner sp(2);
    sp.spin();

    return 0;
}

