#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "serial_comm/serial_comm.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace serial_mul
{
    class serial_ros :public serial_comm
    {
    public:
        serial_ros(std::string serial_port_name, int serial_baudrate, bool uav_terminal,int id): serial_comm(serial_port_name,serial_baudrate),Is_uav_port(uav_terminal),_id(id)
        {
        }
      
        void callbackProcess()
        {
            if(Is_uav_port)
            {
                write_state_data(_state_data);
                read_cmd_data();
                get_read_cmd_data();   
                _pub_cmd.publish(_cmd_vel);
            }
            else
            {
                write_cmd_data(_cmd_data);
                read_state_data();
                get_read_state_data();
                _pub_state_tf.publish(_state);
            }
          
            
        }

        bool setRosCommunication(ros::NodeHandle &nh)
        {
            
            if(Is_uav_port)
            {
                _sub_state_tf = nh.subscribe("truth_state",10,&serial_ros::state_callback,this);
                _pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);
            }
            else
            {
                _sub_cmd = nh.subscribe("cmd_vel",10,&serial_ros::vel_callback,this);
                _pub_state_tf = nh.advertise<nav_msgs::Odometry>("truth_state",10);
            } 
            comm_setup();
            return true;
        }

        void startRosCommunicateMode(ros::NodeHandle &nh)
        {
           boost::function0<void> f = boost::bind(&serial_ros::callbackProcess,this);
           boost::thread thread_viewer(f);
           thread_viewer.detach();
        }

        void vel_callback(const geometry_msgs::Twist::ConstPtr& vel_data)
        {
            std::cout<<*vel_data<<std::endl;
            // *10000   transform double to int, for serial transmission.
            _cmd_data.id = _id;
            _cmd_data.ax = (int32_t)(vel_data->linear.x*10000);
            _cmd_data.ay = (int32_t)(vel_data->linear.y*10000);
            _cmd_data.vz = (int32_t)(vel_data->linear.z*10000);
            _cmd_data.z_rate = (int32_t)(vel_data->angular.z*10000);
        }


        void state_callback(const nav_msgs::Odometry::ConstPtr& state_data)
        {
            _state_data.id = _id;
            _state_data.px = (int32_t)(state_data->pose.pose.position.x*10000);
            _state_data.py = (int32_t)(state_data->pose.pose.position.y*10000);
            _state_data.pz = (int32_t)(state_data->pose.pose.position.z*10000);

            _state_data.qx = (int32_t)(state_data->pose.pose.orientation.x*10000);
            _state_data.qy = (int32_t)(state_data->pose.pose.orientation.y*10000);
            _state_data.qz = (int32_t)(state_data->pose.pose.orientation.z*10000);
            _state_data.qw = (int32_t)(state_data->pose.pose.orientation.w*10000);

            _state_data.vx = (int32_t)(state_data->twist.twist.linear.x*10000);
            _state_data.vy = (int32_t)(state_data->twist.twist.linear.y*10000);
            _state_data.vz = (int32_t)(state_data->twist.twist.linear.z*10000);

        }

        void get_read_cmd_data()
        {
            if(_pubCmdData.id==_id)
            {
                _cmd_vel.linear.x = _pubCmdData.a_x;
                _cmd_vel.linear.y = _pubCmdData.a_y;
                _cmd_vel.linear.z = _pubCmdData.v_z;
                _cmd_vel.angular.x = 0.0;
                _cmd_vel.angular.y = 0.0;
                _cmd_vel.angular.z = _pubCmdData.yaw_rate;
            }
        }

        void get_read_state_data()
        {
            if(_pubStateData.id==_id)
            {
                _state.header.stamp = ros::Time::now();
                _state.header.frame_id = "serial_state_tf";
                _state.pose.pose.position.x = _pubStateData.p_x;
                _state.pose.pose.position.y = _pubStateData.p_y;
                _state.pose.pose.position.z = _pubStateData.p_z;

                _state.pose.pose.orientation.x = _pubStateData.q_x;
                _state.pose.pose.orientation.y = _pubStateData.q_y;
                _state.pose.pose.orientation.z = _pubStateData.q_z;
                _state.pose.pose.orientation.w = _pubStateData.q_w;

                _state.twist.twist.linear.x = _pubStateData.v_x;
                _state.twist.twist.linear.y = _pubStateData.v_y;
                _state.twist.twist.linear.z = _pubStateData.v_z;

            }
                        
        }

    private:
        // matlab
        ros::Subscriber _sub_cmd;
        ros::Publisher _pub_state_tf;

        // uav
        ros::Subscriber _sub_state_tf;
        ros::Publisher _pub_cmd;

        // ros::NodeHandle _nh;
        geometry_msgs::Twist _cmd_vel;
        nav_msgs::Odometry _state;
        int _id;
        double Hz;
        bool Is_uav_port;
        
    };
}



