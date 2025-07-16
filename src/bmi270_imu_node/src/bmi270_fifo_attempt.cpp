/**\
 * This code makes use of the Bosch code, released with the following license.
 *
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/
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
// Although bmi2.h often pulls it in, explicitly including for definitions like bmi2_sens_data can sometimes help with visibility.
// #include "bmi270_driver/bmi2_defs.h"

/******************************************************************************/
/*!                  Macros                                                   */
// From fifo_full_header_mode example

/*! Buffer size allocated to store raw FIFO data. */
#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)

/*! Length of data to be read from FIFO. */
#define BMI2_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

/*! Number of accel frames to be extracted from FIFO. */

/*! Calculation for frame count: Total frame count = Fifo buffer size(2048)/ Total frames(6 Accel, 6 Gyro and 1 header,
 * totaling to 13) which equals to 157.
 *
 * Extra frames to parse sensortime data
 */
#define BMI2_FIFO_ACCEL_FRAME_COUNT     UINT8_C(185)

/*! Number of gyro frames to be extracted from FIFO. */
#define BMI2_FIFO_GYRO_FRAME_COUNT      UINT8_C(185)

/*! Macro to read sensortime byte in FIFO. */
#define SENSORTIME_OVERHEAD_BYTE        UINT8_C(220)

/* To read sensortime, extra 3 bytes are added to fifo buffer. */
// TODO: needed?
uint16_t fifo_buffer_size = BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE;

/* Number of bytes of FIFO data
 * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
 * Array size same as fifo_buffer_size
 */
uint8_t fifo_data[BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE];

/* Array of accelerometer frames -> Total bytes =
* 157 * (6 axes + 1 header bytes) = 1099 bytes */
struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

/* Array of gyro frames -> Total bytes =
 * 157 * (6 axes + 1 header bytes) = 1099 bytes */
struct bmi2_sens_axes_data fifo_gyro_data[BMI2_FIFO_GYRO_FRAME_COUNT] = { { 0 } };

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
        timer_ = create_wall_timer(2ms, std::bind(&BMI270Node::timer_callback, this));

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
        /* Status of api are returned to this variable. */
        int8_t rslt;

        uint16_t index = 0;
        uint16_t fifo_length = 0;
        uint16_t config = 0;

        /* Variable to get fifo full interrupt status. */
        uint16_t int_status = 0;

        uint16_t accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;

        uint16_t gyro_frame_length = BMI2_FIFO_GYRO_FRAME_COUNT;

        int8_t try = 1;

        /* Sensor initialization configuration. */
        struct bmi2_dev bmi2_dev;

        /* Initialize FIFO frame structure. */
        struct bmi2_fifo_frame fifoframe = { 0 };

        /* Accel and gyro sensor are listed in array. */
        uint8_t sensor_sel[2] = { BMI2_ACCEL, BMI2_GYRO };




        bmi2_dev.intf = BMI2_I2C_INTF; // Set interface to I2C
        bmi2_dev.intf_ptr = &i2c_fd_;  // Pointer to the I2C file descriptor
        bmi2_dev.read = bmi2_i2c_read; // Assign custom I2C read function
        bmi2_dev.write = bmi2_i2c_write; // Assign custom I2C write function
        bmi2_dev.delay_us = bmi2_delay_us; // Assign custom delay function
        bmi2_dev.read_write_len = 32; // Maximum bytes to read/write in a single transaction

        // Initialize BMI270 sensor
        rslt = bmi270_init(&bmi2_dev);
        if (rslt != BMI2_OK) {
            RCLCPP_FATAL(get_logger(), "BMI270 initialization failed. Check wiring and power.");
            bmi2_error_codes_print_result(rslt); //TODO may not work, adjust to ros2 log
            close(i2c_fd_);
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "BMI270 initialized successfully.");

        //New from fifo_full_header_mode
        /* Configuration settings for accel and gyro. */
        rslt = set_accel_gyro_config(&bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* NOTE:
         * Accel and Gyro enable must be done after setting configurations
         */
        rslt = bmi270_sensor_enable(sensor_sel, 2, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Before setting FIFO, disable the advance power save mode. */
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Initially disable all configurations in fifo. */
        rslt = bmi2_get_fifo_config(&config, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Initially disable all configurations in fifo. */
        rslt = bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Update FIFO structure. */
        /* Mapping the buffer to store the fifo data. */
        fifoframe.data = fifo_data;

        /* Length of FIFO frame. */
        /* To read sensortime, extra 3 bytes are added to fifo user length. */
        fifoframe.length = BMI2_FIFO_RAW_DATA_USER_LENGTH + SENSORTIME_OVERHEAD_BYTE;

        /* Set FIFO configuration by enabling accel, gyro and timestamp.
         * NOTE 1: The header mode is enabled by default.
         * NOTE 2: By default the FIFO operating mode is in FIFO mode.
         * NOTE 3: Sensortime is enabled by default */
        printf("FIFO is configured in header mode\n");
        rslt = bmi2_set_fifo_config(BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN, BMI2_ENABLE, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        /* Map FIFO full interrupt. */
        fifoframe.data_int_map = BMI2_FFULL_INT;
        rslt = bmi2_map_data_int(fifoframe.data_int_map, BMI2_INT1, &bmi2_dev);
        bmi2_error_codes_print_result(rslt);

        uint8_t sensortime_raw[3] = { 0 };
        rslt = bmi2_get_regs(BMI2_CHIP_ID_ADDR, sensortime_raw, 3, &bmi2_dev);
        if (rslt == BMI2_OK) {
            uint32_t sensortime = 0;
            sensortime = (uint32_t)sensortime_raw[0] | ((uint32_t)sensortime_raw[1] << 8) | ((uint32_t)sensortime_raw[2] << 16);
            printf("sensor time (s) %f\n", sensortime * BMI2_SENSORTIME_RESOLUTION);
        }



        // previous
        
        // Sensor configuration
//        struct bmi2_sens_config config[2] = {0}; // Initialize with zeros
//
//        // Accelerometer configuration
//        config[0].type = BMI2_ACCEL;
//        config[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ; // Output Data Rate: 100Hz
//        config[0].cfg.acc.range = BMI2_ACC_RANGE_4G; // Range: +/- 4G
//        config[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4; // Bandwidth parameter: Normal average 4
//        config[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // Performance mode: Optimized
//
//        // Gyroscope configuration
//        // all hardcoded for now, TODO see how to use a parameter file to set them 
//        config[1].type = BMI2_GYRO;
//        config[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ; // Output Data Rate: 100Hz
//        config[1].cfg.gyr.range = BMI2_GYR_RANGE_2000; // Range: +/- 2000 degrees per second (dps)
//        config[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE; // Bandwidth parameter: Normal mode
//        config[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE; // Noise performance mode: Optimized
//        config[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // Filter performance mode: Optimized
//
//        // Set sensor configurations
//        if (bmi2_set_sensor_config(config, 2, &bmi2_dev) != BMI2_OK) {
//            RCLCPP_FATAL(get_logger(), "Sensor configuration failed.");
//            close(i2c_fd_);
//            rclcpp::shutdown();
//            return;
//        }
//        RCLCPP_INFO(get_logger(), "Sensor configured for Accel and Gyro.");
//
//        // Enable sensors
//        uint8_t sensors[2] = { BMI2_ACCEL, BMI2_GYRO };
//        if (bmi2_sensor_enable(sensors, 2, &bmi2_dev) != BMI2_OK) {
//            RCLCPP_FATAL(get_logger(), "Sensor enabling failed.");
//            close(i2c_fd_);
//            rclcpp::shutdown();
//            return;
//        }
        RCLCPP_INFO(get_logger(), "Accel and Gyro enabled.");
    } // node declaration

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
        // struct bmi2_sens_data sensor_data; // no more needed

        // Read sensor data from BMI270
        // bmi2_get_sensor_data expects a pointer to a single bmi2_sens_data struct and the device struct
        // if ((bmi2_get_sensor_data(&sensor_data, &bmi2_dev) == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) && (sensor_data.status & BMI2_DRDY_GYR)) {
        if ((rslt == BMI2_OK) && (int_status & BMI2_FFULL_INT_STATUS_MASK))
            // TODO: continue ffrom here!

            // Create and populate ROS2 Imu message
            auto msg = sensor_msgs::msg::Imu();
            struct timespec tp;
            clock_gettime(CLOCK_MONOTONIC, &tp);
            msg.header.stamp = rclcpp::Time(tp.tv_sec * 1000000000 + tp.tv_nsec, RCL_STEADY_TIME);
            msg.header.frame_id = "imu_link"; // Coordinate frame ID
            // TODO: populate sensor data from fifo_accel_data (see example, for loop)                                              

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
    struct bmi2_dev bmi2_dev; // Bosch sensor device structure
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

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmi2_error_codes_print_result(int8_t rslt)
{
    switch (rslt)
    {
        case BMI2_OK:

            /* Do nothing */
            break;

        case BMI2_W_FIFO_EMPTY:
            printf("Warning [%d] : FIFO empty\r\n", rslt);
            break;
        case BMI2_W_PARTIAL_READ:
            printf("Warning [%d] : FIFO partial read\r\n", rslt);
            break;
        case BMI2_E_NULL_PTR:
            printf(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI2_E_COM_FAIL:
            printf(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI2_E_DEV_NOT_FOUND:
            printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI2_E_INVALID_SENSOR:
            printf(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_FAIL:
            printf(
                "Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INT_PIN:
            printf(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI2_E_OUT_OF_RANGE:
            printf(
                "Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
                rslt);
            break;

        case BMI2_E_ACC_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
                rslt);
            break;

        case BMI2_E_GYRO_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
                rslt);
            break;

        case BMI2_E_ACC_GYR_INVALID_CFG:
            printf(
                "Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
                rslt);
            break;

        case BMI2_E_CONFIG_LOAD:
            printf(
                "Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_PAGE:
            printf(
                "Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
                rslt);
            break;

        case BMI2_E_SET_APS_FAIL:
            printf(
                "Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
                rslt);
            break;

        case BMI2_E_AUX_INVALID_CFG:
            printf(
                "Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
                rslt);
            break;

        case BMI2_E_AUX_BUSY:
            printf(
                "Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
                rslt);
            break;

        case BMI2_E_REMAP_ERROR:
            printf(
                "Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
                rslt);
            break;

        case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
            printf(
                "Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
                rslt);
            break;

        case BMI2_E_SELF_TEST_NOT_DONE:
            printf(
                "Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_INPUT:
            printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI2_E_INVALID_STATUS:
            printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI2_E_CRT_ERROR:
            printf("Error [%d] : CRT error. It occurs when the CRT test has failed\r\n", rslt);
            break;

        case BMI2_E_ST_ALREADY_RUNNING:
            printf(
                "Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
            printf(
                "Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
                rslt);
            break;

        case BMI2_E_DL_ERROR:
            printf(
                "Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
                rslt);
            break;

        case BMI2_E_PRECON_ERROR:
            printf(
                "Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
                rslt);
            break;

        case BMI2_E_ABORT_ERROR:
            printf("Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n", rslt);
            break;

        case BMI2_E_WRITE_CYCLE_ONGOING:
            printf(
                "Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
                rslt);
            break;

        case BMI2_E_ST_NOT_RUNING:
            printf(
                "Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
                rslt);
            break;

        case BMI2_E_DATA_RDY_INT_FAILED:
            printf(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI2_E_INVALID_FOC_POSITION:
            printf(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        default:
            printf("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
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

