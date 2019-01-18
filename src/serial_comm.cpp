#include "serial_comm/serial_comm.h"

namespace serial_mul
{
serial_comm::serial_comm(std::string serial_port,  int baudrate)
{
    _device_name = serial_port;
    _serial_baudrate = baudrate;

    // _device_name = "/dev/ttyUSB0";
    // _serial_baudrate = "B57600";

    _data_cmd_len = sizeof(data_cmd_t);
    _data_state_len = sizeof(data_state_t);
    _header = 0xA5;
    _tail = 0xFE;

}

serial_comm::~serial_comm()
{
  close(_fd);
}

bool serial_comm::comm_setup()
{
  // fd = open (dev.c_str(), O_RDWR);
  _fd = open (_device_name.c_str(),O_RDWR|O_NOCTTY|O_NDELAY);
  sleep(1);
  if(_fd==-1)
  {
    //perror("error!");
    return false;
  }
  struct termios oldtio;
  tcgetattr(_fd,&oldtio);
  if(tcgetattr(_fd,&oldtio)!=0)
  {
    //perror("error!");
    return false;
  }
  struct termios newtio;
  newtio.c_cflag|=CRTSCTS|CS8|CLOCAL|CREAD;
  newtio.c_cflag&=~PARODD;
  newtio.c_cflag&=~CSTOPB;
  // newtio.c_cflag=CLOCAL|CREAD;
  cfsetispeed(&newtio,_serial_baudrate);
  newtio.c_iflag=IGNPAR;
  newtio.c_oflag=0;
  newtio.c_lflag=0;
  newtio.c_cc[VMIN]=1;
  // newtio.c_cc[VMIN]=0;
  newtio.c_cc[VTIME]=10;
  tcflush(_fd,TCIFLUSH);
  tcsetattr(_fd,TCSANOW,&newtio);
  return true;
}

void serial_comm::write_state_data(data_state_t state_data_to_write)
{
    state_data_to_write.sof = _header;
    state_data_to_write.end = _tail;
    uint8_t buff[_data_state_len];
    memset(buff, 0, _data_state_len);
    memcpy(buff, &state_data_to_write, sizeof(data_state_t));
    write(_fd, buff, sizeof(data_state_t));
}

void serial_comm::write_cmd_data(data_cmd_t cmd_data_to_write)
{
  cmd_data_to_write.sof = _header;
  cmd_data_to_write.end = _tail;
  uint8_t buff[_data_cmd_len];
  memset(buff, 0, _data_cmd_len);
  memcpy(buff, &cmd_data_to_write, sizeof(data_cmd_t));
  write(_fd, buff, sizeof(data_cmd_t));
}

void serial_comm::read_cmd_data()
{
    uint8_t buff[_data_cmd_len];
    memset(buff, 0, _data_cmd_len);
    read(_fd,buff,sizeof(data_cmd_t));
    memcpy(&_cmd_data, buff, sizeof(data_cmd_t));
    if(_cmd_data.sof==_header && _cmd_data.end == _tail)
    {
        _pubCmdData.id = _cmd_data.id;
        _pubCmdData.a_x = (double)_cmd_data.ax/10000.0;
        _pubCmdData.a_y = (double)_cmd_data.ay/10000.0;
        _pubCmdData.v_z = (double)_cmd_data.vz/10000.0;
        _pubCmdData.yaw_rate = (double)_cmd_data.z_rate/10000.0;
    }
}

void serial_comm::read_state_data()
{
    uint8_t buff[_data_state_len];
    memset(buff, 0, _data_state_len);
    read(_fd,buff,sizeof(data_state_t));
    memcpy(&_state_data, buff, sizeof(data_state_t));
    if(_state_data.sof==_header && _state_data.end == _tail)
    {
       
        _pubStateData.id = _state_data.id;

        _pubStateData.p_x = (double)_state_data.px/10000.0;
        _pubStateData.p_y = (double)_state_data.py/10000.0;
        _pubStateData.p_z = (double)_state_data.pz/10000.0;

        _pubStateData.q_x = (double)_state_data.qx/10000.0;
        _pubStateData.q_y = (double)_state_data.qy/10000.0;
        _pubStateData.q_z = (double)_state_data.qz/10000.0;
        _pubStateData.q_w = (double)_state_data.qw/10000.0;

        _pubStateData.v_x = (double)_state_data.vx/10000.0;
        _pubStateData.v_y = (double)_state_data.vy/10000.0;
        _pubStateData.v_z = (double)_state_data.vz/10000.0;
        std::cout<<_pubStateData<<std::endl;

    }
}

}//namespace serial_comm

