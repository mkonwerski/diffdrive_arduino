#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>


#include <unistd.h>        //Needed for I2C port
#include <fcntl.h>          //Needed for I2C port
#include <sys/ioctl.h>      //Needed for I2C port
#include <linux/i2c-dev.h>  //Needed for I2C port
#include <linux/i2c.h>      //Needed for I2C port

#include <iostream>
#include <iomanip>
#include <string>
#include <cerrno>
#include <cstdint>
#include <cstring>

const std::string i2c_filename = "/dev/i2c-1";
const int i2c_addr = 0x41;       //<<<<<The I2C address of the slave

int fd_i2c;

int i2c_rdwr_block(int fd, uint8_t read_write, uint8_t reg, unsigned char* buffer)
{
    int rv;
    uint8_t length = 4;
    union i2c_smbus_data smbus_data;
    struct i2c_smbus_ioctl_data ioctl_data;

    smbus_data.block[0] = length;

    if ( read_write != I2C_SMBUS_READ )
    {
        for(int i = 0; i < length; i++)
        {
            smbus_data.block[i + 1] = buffer[i];
        }
    }

    ioctl_data.read_write = read_write;
    ioctl_data.command = reg;
    ioctl_data.size = I2C_SMBUS_I2C_BLOCK_DATA;
    ioctl_data.data = &smbus_data;

    rv = ioctl (fd, I2C_SMBUS, &ioctl_data);
    if (rv < 0)
    {
        std::cerr << "Accessing I2C Read/Write failed! Error is: " << strerror(errno) << std::endl;
        return -1;
    }

    if (read_write == I2C_SMBUS_READ)
    {
        for(int i = 0; i < length; i++)
        {
            buffer[i] = smbus_data.block[i+1];
        }
    }
    return rv;
}

void ArduinoComms::setup()
{    
    int fd;
    int rv;

    if ((fd = open(i2c_filename.c_str(), O_RDWR)) < 0)
    {
        std::cout << "Failed to open the i2c bus. Error code: " << fd << std::endl;
    }

    if ((rv = ioctl(fd, I2C_SLAVE, i2c_addr)) < 0)
    {
        std::cout << "Failed to acquire bus access and/or talk to slave. Error code: " << rv << std::endl;
    }
    fd_i2c = fd;
}

void ArduinoComms::sendEmptyMsg()
{
    sendMsg(82,0,0);
    sendMsg(82,1,0);
}

void ArduinoComms::readEncoderValues(int val_1, int val_2)
{     
    unsigned char buffer[1] = {0};
    i2c_rdwr_block(fd_i2c, I2C_SMBUS_READ, 14, buffer);
    std::string value1(1, buffer[0]);
    val_1 = std::atoi(value1.c_str());

   std::cout << "readEncoderValues: " << val_1 << std::endl;

    i2c_rdwr_block(fd_i2c, I2C_SMBUS_READ, 15, buffer);
    std::string value2(1, buffer[0]);
    val_1 = std::atoi(value2.c_str());
    std::cout << "readEncoderValues: " << val_2 << std::endl;
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    sendMsg( 82, 0, val_1);
    sendMsg( 82, 1, val_2);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
}

void ArduinoComms::sendMsg(int val_1, int chanel, int value)
{
    //unsigned char values[0] = {0};
    unsigned char values[0];
    values[0] = chanel;
    values[1] = value;

    i2c_rdwr_block(fd_i2c, I2C_SMBUS_WRITE, val_1, values);     

   // return response;
}
