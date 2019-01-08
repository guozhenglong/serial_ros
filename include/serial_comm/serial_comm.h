#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
// #include <std_msgs/String.h>
#include <serial_ros/Cmd_uav.h>
#include <serial_ros/State_uav.h>
#include <fcntl.h>      /*file control lib*/
#include <termios.h>    /*PPSIX terminal*/
#include <errno.h>      /*error information*/
#include <stdlib.h>     /*standard lib*/
#include <unistd.h>     /*Unix standard func*/

#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <memory>
#include <vector>


namespace serial_mul
{
typedef struct
{
    uint8_t sof;
    int32_t ax;
    int32_t ay;
    int32_t vz;
    int32_t z_rate;
    uint8_t end;

}__attribute__((packed)) data_cmd_t;


typedef struct
{
    uint8_t sof;
    int32_t px;
    int32_t py;
    int32_t pz;

    int32_t qx;
    int32_t qy;
    int32_t qz;
    int32_t qw;

    int32_t vx;
    int32_t vy;
    int32_t vz;
    uint8_t end;

}__attribute__((packed)) data_state_t;


class serial_comm{
public:
  serial_comm(std::string serial_port,  int baudrate);

  ~serial_comm();

  /**
   * @brief Initialization the serial paramters for writing data
   * @return
   */
  bool comm_setup();

  /**
   * @brief Writing data to serial
   */
  void write_state_data(data_state_t state_data_to_write);
  void write_cmd_data(data_cmd_t cmd_data_to_write);
  

     /**
   * @brief reading data from serial
   */
  void read_state_data();
  void read_cmd_data();

public:
  data_cmd_t _cmd_data;
  data_state_t _state_data;
  serial_ros::Cmd_uav _pubCmdData;
  serial_ros::State_uav _pubStateData;

private:
  std::string _device_name;
  int _serial_baudrate;
  // std_msgs::String pubData;
  int _fd;
  int _data_cmd_len, _data_state_len;
  uint8_t _header, _tail;



};
}

#endif // SERIAL_COMM_H
