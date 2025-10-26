#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

class LSM6DS3_ROS {
private:
    ros::NodeHandle nh_;
    ros::Publisher imu_pub_;
    ros::Timer timer_;
    
    int spi_fd_;
    uint32_t speed_ = 1000000;
    uint8_t mode_ = SPI_MODE_3;
    std::string spi_device_;

    // Register addresses
    static constexpr uint8_t WHO_AM_I = 0x0F;
    static constexpr uint8_t CTRL1_XL = 0x10;
    static constexpr uint8_t CTRL2_G = 0x11;
    static constexpr uint8_t OUTX_L_XL = 0x28;
    static constexpr uint8_t OUTX_L_G = 0x22;
    
    static constexpr uint8_t READ_CMD = 0x80;
    static constexpr uint8_t WRITE_CMD = 0x00;

public:
    LSM6DS3_ROS() {
        // Get SPI device from parameter server with private node handle
        ros::NodeHandle private_nh("~");
        private_nh.param("spi_device", spi_device_, std::string("/dev/spidev0.0"));
        
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
        
        ROS_INFO("Using SPI device: %s", spi_device_.c_str());
        
        if (!initializeSPI(spi_device_)) {
            ROS_ERROR("Failed to initialize SPI");
            return;
        }
        
        if (!initializeSensor()) {
            ROS_ERROR("Failed to initialize sensor");
            return;
        }
        
        timer_ = nh_.createTimer(ros::Duration(0.02), &LSM6DS3_ROS::publishIMU, this);
        ROS_INFO("LSM6DS3 IMU publisher started");
    }

    ~LSM6DS3_ROS() {
        if (spi_fd_ >= 0) close(spi_fd_);
    }

private:
    bool initializeSPI(const std::string& device) {
        spi_fd_ = open(device.c_str(), O_RDWR);
        if (spi_fd_ < 0) {
            ROS_ERROR("Cannot open SPI device: %s (errno: %d)", device.c_str(), errno);
            return false;
        }
        
        if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode_) == -1) {
            ROS_ERROR("Cannot set SPI mode");
            return false;
        }
        
        if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) == -1) {
            ROS_ERROR("Cannot set SPI speed");
            return false;
        }
        
        ROS_INFO("SPI initialized successfully");
        return true;
    }

	bool initializeSensor() {
		uint8_t whoami = readRegister(WHO_AM_I);
		ROS_INFO("WHO_AM_I register: 0x%02X", whoami);

		if (whoami != 0x69) {
			ROS_ERROR("WHO_AM_I check failed. Expected 0x69, got 0x%02X", whoami);
			ROS_ERROR("This usually indicates wiring or power issues");
			return false;
		}

		// Configure accelerometer and gyroscope
		writeRegister(CTRL1_XL, 0x40); // 104 Hz, ±4g
		writeRegister(CTRL2_G, 0x40);  // 104 Hz, ±500 dps

		usleep(100000);
		ROS_INFO("LSM6DS3 sensor initialized successfully");
		return true;
	}

    uint8_t readRegister(uint8_t reg) {
        uint8_t tx[2] = {static_cast<uint8_t>(reg | READ_CMD), 0};
        uint8_t rx[2] = {0};
        spiTransfer(tx, rx, 2);
        return rx[1];
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t tx[2] = {static_cast<uint8_t>(reg | WRITE_CMD), value};
        uint8_t rx[2] = {0};
        spiTransfer(tx, rx, 2);
    }

    int16_t read16Bit(uint8_t reg) {
        uint8_t tx[3] = {static_cast<uint8_t>(reg | READ_CMD), 0, 0};
        uint8_t rx[3] = {0};
        spiTransfer(tx, rx, 3);
        return static_cast<int16_t>((rx[2] << 8) | rx[1]);
    }

    void publishIMU(const ros::TimerEvent& e) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu";

        try {
            // Read accelerometer data (convert from mg to m/s²)
            int16_t ax = read16Bit(OUTX_L_XL);
            int16_t ay = read16Bit(OUTX_L_XL + 2);
            int16_t az = read16Bit(OUTX_L_XL + 4);

            // Read gyroscope data (convert from mdps to rad/s)
            int16_t gx = read16Bit(OUTX_L_G);
            int16_t gy = read16Bit(OUTX_L_G + 2);
            int16_t gz = read16Bit(OUTX_L_G + 4);

            // Convert accelerometer (0.122 mg/LSB * 0.001 g/mg * 9.80665 m/s²/g)
            imu_msg.linear_acceleration.x = ax * 0.122 * 0.001 * 9.80665;
            imu_msg.linear_acceleration.y = ay * 0.122 * 0.001 * 9.80665;
            imu_msg.linear_acceleration.z = az * 0.122 * 0.001 * 9.80665;

            // Convert gyroscope (17.50 mdps/LSB * 0.001 dps/mdps * π/180 rad/dps)
            imu_msg.angular_velocity.x = gx * 17.50 * 0.001 * M_PI / 180.0;
            imu_msg.angular_velocity.y = gy * 17.50 * 0.001 * M_PI / 180.0;
            imu_msg.angular_velocity.z = gz * 17.50 * 0.001 * M_PI / 180.0;

            // Set orientation to identity
            imu_msg.orientation.w = 1.0;

            // Set covariance
            imu_msg.orientation_covariance[0] = -1;
            imu_msg.angular_velocity_covariance[0] = 0.02;
            imu_msg.angular_velocity_covariance[4] = 0.02;
            imu_msg.angular_velocity_covariance[8] = 0.02;
            imu_msg.linear_acceleration_covariance[0] = 0.04;
            imu_msg.linear_acceleration_covariance[4] = 0.04;
            imu_msg.linear_acceleration_covariance[8] = 0.04;

            imu_pub_.publish(imu_msg);

        } catch (const std::exception& e) {
            ROS_WARN("Failed to read IMU data: %s", e.what());
        }
    }

    void spiTransfer(uint8_t* tx, uint8_t* rx, int len) {
        struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr));
        
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = len;
        tr.speed_hz = speed_;
        tr.bits_per_word = 8;
        
        if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
            throw std::runtime_error("SPI transfer failed");
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu");
    
    ROS_INFO("Starting LSM6DS3 IMU publisher for RPi 5");
    
    LSM6DS3_ROS imu;
    
    if (ros::ok()) {
        ros::spin();
    }
    
    return 0;
}
