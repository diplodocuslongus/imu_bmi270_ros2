#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <chrono>
#include <fcntl.h>        // For open()
#include <unistd.h>       // For close(), read(), write()
#include <linux/i2c-dev.h> // For I2C_SLAVE
#include <sys/ioctl.h>    // For ioctl()
#include <cmath>          // For M_PI
#include <string.h>       // For memcpy
#include <thread>         // For std::this_thread::sleep_for
#include <errno.h>        // For strerror(errno)
#include <time.h>

// Include Bosch BMI270 API headers
//  paths  to match project structure, 
//  see bmi270_driver package.xml and CMakeLists.txt
#include "bmi270_driver/bmi2.h"
#include "bmi270_driver/bmi270.h"

using namespace std::chrono_literals;

// Forward declarations for I2C and delay functions
// These must match the function pointer types expected by the Bosch API (was
// the source of many build error...)
int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void*); // void* parameter is required by API but unused here

class BMI270Node : public rclcpp::Node
{
public:
    BMI270Node() : Node("bmi270_node")
    {
        // Initialize ROS2 publisher and timer
        publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        // adjust the time to the ODR
        // timer_ = create_wall_timer(2ms, std::bind(&BMI270Node::timer_callback, this));
        timer_ = create_wall_timer(200us, std::bind(&BMI270Node::timer_callback, this));

        // Open I2C device
        // set the bus here (defined on the pi at /boot/firmware/config.txt as a dto)
        i2c_fd_ = open("/dev/i2c-3", O_RDWR); // change e.g. i2c-3`
        if (i2c_fd_ < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to open I2C device /dev/i2c-1. Error: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        // Set I2C slave address for BMI270 (0x68 or 0x69)
        // Ensure this matches the hardware configuration. 
        // BMI2_I2C_ADDR_PRIM is typically 0x68.
        // I hard code 0x68 to avoid "not declared in this scope" issues 
        if (ioctl(i2c_fd_, I2C_SLAVE, 0x68) < 0) {
            RCLCPP_FATAL(get_logger(), "Failed to set I2C address 0x%X. Please check your I2C address. Error: %s", 0x68, strerror(errno));
            close(i2c_fd_); // Close the file descriptor if ioctl fails
            rclcpp::shutdown();
            return;
        }

        // Initialize BMI2 sensor device structure
        dev_.intf = BMI2_I2C_INTF; // Set interface to I2C
        dev_.intf_ptr = &i2c_fd_;  // Pointer to the I2C file descriptor
        dev_.read = bmi2_i2c_read; // Assign custom I2C read function
        dev_.write = bmi2_i2c_write; // Assign custom I2C write function
        dev_.delay_us = bmi2_delay_us; // Assign custom delay function
        dev_.read_write_len = 32; // Maximum bytes to read/write in a single transaction
        dev_.config_file_ptr = NULL; // Assign to NULL to load the default config file. see common.c

        // Initialize BMI270 sensor
        if (bmi270_init(&dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "BMI270 initialization failed. Check wiring and power.");
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "BMI270 initialized successfully.");

        // Sensor configuration
        struct bmi2_sens_config config[2] = {0}; // Initialize with zeros

        // Accelerometer configuration
        config[0].type = BMI2_ACCEL;
        config[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ; // Output Data Rate defined in bmi2_defs.h
        config[0].cfg.acc.range = BMI2_ACC_RANGE_4G; // Range: +/- 4G
        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4; // Bandwidth parameter: Normal average 4
        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // Performance mode: Optimized

        // Gyroscope configuration
        // all hardcoded for now, TODO see how to use a parameter file to set them 
        config[1].type = BMI2_GYRO;
        config[1].cfg.gyr.odr = BMI2_GYR_ODR_800HZ; // Output Data Rate defined in bmi2_defs.h
        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000; // Range: +/- 2000 degrees per second (dps)
        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE; // Bandwidth parameter: Normal mode
        config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE; // Noise performance mode: Optimized
        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // Filter performance mode: Optimized

        // Set sensor configurations
        if (bmi2_set_sensor_config(config, 2, &dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "Sensor configuration failed.");
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Sensor configured for Accel and Gyro.");

        // Enable sensors
        uint8_t sensors[2] = { BMI2_ACCEL, BMI2_GYRO };
        if (bmi2_sensor_enable(sensors, 2, &dev_) != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "Sensor enabling failed.");
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Accel and Gyro enabled.");
    }

    // Destructor to ensure I2C file descriptor is closed
    ~BMI270Node()
    {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            RCLCPP_INFO(get_logger(), "I2C device closed.");
        }
    }

private:
    void timer_callback()
    {
        // Declaring a single bmi2_sens_data struct
        // as bmi2_get_sensor_data populates it with all enabled sensor data
        struct bmi2_sens_data sensor_data;

        // Read sensor data from BMI270
        // bmi2_get_sensor_data expects a pointer to a single bmi2_sens_data struct and the device struct
        if ((bmi2_get_sensor_data(&sensor_data, &dev_) == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) && (sensor_data.status & BMI2_DRDY_GYR)) {

            // Create and populate ROS2 Imu message
            auto msg = sensor_msgs::msg::Imu();
            struct timespec tp;
            clock_gettime(CLOCK_MONOTONIC, &tp);
            msg.header.stamp = rclcpp::Time(tp.tv_sec * 1000000000 + tp.tv_nsec, RCL_STEADY_TIME);
            msg.header.frame_id = "imu_link"; // Coordinate frame ID

            // Convert raw accelerometer data to m/s^2
            // Accessing acc data directly from the sensor_data struct
            float acc_scale = (4.0f * 9.80665f) / 32768.0f; // Scale factor for +/-4G range (m/s^2 per LSB)
            msg.linear_acceleration.x = sensor_data.acc.x * acc_scale;
            msg.linear_acceleration.y = sensor_data.acc.y * acc_scale;
            msg.linear_acceleration.z = sensor_data.acc.z * acc_scale;

            // Convert raw gyroscope data to rad/s
            // Accessing gyr data directly from the sensor_data struct
            float gyro_scale = (2000.0f / 32768.0f) * (M_PI / 180.0f); // Scale factor for +/-2000 dps range (rad/s per LSB)
            msg.angular_velocity.x = sensor_data.gyr.x * gyro_scale;
            msg.angular_velocity.y = sensor_data.gyr.y * gyro_scale;
            msg.angular_velocity.z = sensor_data.gyr.z * gyro_scale;

            // The following is according to ros2 imu message:
            // Gyroscope and Accelerometer covariance are usually set to 0 if not calculated
            // If you have calibration data, you can set these. Otherwise, leave as 0
            msg.angular_velocity_covariance[0] = -1; // Indicate no covariance data available
            msg.linear_acceleration_covariance[0] = -1; // Indicate no covariance data available

            // Publish the Imu message
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i2c_fd_; // File descriptor for the I2C bus
    struct bmi2_dev dev_; // Bosch sensor device structure
};

/**
 * @brief Custom I2C read function for BMI270 API.
 * This function reads a specified number of bytes from the sensor via I2C.
 *
 * @param reg_addr   : Register address to start reading from.
 * @param data       : Pointer to buffer to store read data.
 * @param len        : Number of bytes to read.
 * @param intf_ptr   : Pointer to I2C file descriptor.
 * @return BMI2_OK on success, BMI2_E_COM_FAIL on failure.
 */
int8_t bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    // Cast the void pointer back to an int pointer to get the file descriptor
    int fd = *(int *)intf_ptr;

    // Write the register address to the I2C device
    if (write(fd, &reg_addr, 1) != 1) {
        // If the write operation fails, return communication failure error
        return BMI2_E_COM_FAIL;
    }

    // Read 'len' bytes from the I2C device into the data buffer
    if (read(fd, data, len) != (int)len) {
        // If the read operation fails or doesn't read the expected number of bytes, return communication failure error
        return BMI2_E_COM_FAIL;
    }

    // Return success
    return BMI2_OK;
}

/**
 * @brief Custom I2C write function for BMI270 API.
 * This function writes a specified number of bytes to the sensor via I2C.
 *
 * @param reg_addr   : Register address to start writing to.
 * @param data       : Pointer to data buffer to write.
 * @param len        : Number of bytes to write.
 * @param intf_ptr   : Pointer to I2C file descriptor.
 * @return BMI2_OK on success, BMI2_E_COM_FAIL on failure.
 */
int8_t bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    // Cast the void pointer back to an int pointer to get the file descriptor
    int fd = *(int *)intf_ptr;

    // Create a buffer to hold the register address followed by the data
    // The I2C write typically sends the register address first, then the data.
    // The size of the buffer is 'len' (for data) + 1 (for register address).
    uint8_t buf[len + 1];

    // Place the register address at the beginning of the buffer
    buf[0] = reg_addr;
    // Copy the data to be written into the buffer, starting after the register address
    memcpy(&buf[1], data, len);

    // Write the entire buffer (register address + data) to the I2C device
    // The total bytes to write is 'len' + 1.
    if (write(fd, buf, len + 1) != (int)(len + 1)) {
        // If the write operation fails or doesn't write the expected number of bytes, return communication failure error
        return BMI2_E_COM_FAIL;
    }

    // Return success
    return BMI2_OK;
}

/**
 * @brief Custom delay function for BMI270 API.
 * This function provides a microsecond delay.
 *
 * @param period   : Delay period in microseconds.
 * @param unused_ptr : Void pointer (unused but required by API signature).
 */
void bmi2_delay_us(uint32_t period, void* unused_ptr)
{
    // Use C++ standard library for sleeping for a specified duration
    std::this_thread::sleep_for(std::chrono::microseconds(period));
}

int main(int argc, char *argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create and spin the BMI270Node
    // std::make_shared ensures proper memory management
    rclcpp::spin(std::make_shared<BMI270Node>());

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}

