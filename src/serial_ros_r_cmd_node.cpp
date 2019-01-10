#include "serial_ros_r_cmd.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_comm_ros");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string serial_port, serial_port_default;
    serial_port_default="/dev/ttyUSB0";
    int baud;
    bool is_uav_terminal;// true-uav; false - matlab
    int Id;
    double Hz;
  

    pnh.param("serial_port_name", serial_port, serial_port_default); 
    // pnh.param("baudrate", baud, B9600); 
    pnh.param("Hz", Hz, 10.0);
    pnh.param("Id", Id, 1); 
    pnh.param("is_uav_terminal", is_uav_terminal, true); 
    baud = B57600;
    serial_mul::serial_ros serial_comm_zigbee(serial_port,baud,is_uav_terminal,Id);
    ros::Rate loopRate(Hz);
    serial_comm_zigbee.setRosCommunication(nh);
    // serial_comm_zigbee.startRosCommunicateMode(nh);
    while(nh.ok())
    {
        serial_comm_zigbee.callbackProcess();
        ros::spinOnce();
        loopRate.sleep();
    }
}