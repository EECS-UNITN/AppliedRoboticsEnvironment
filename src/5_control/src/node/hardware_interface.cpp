/*--------------------------------------------------------------------------*\
 |                                                                          |
 |  Copyright (C) 2019                                                      |
 |                                   .__  __                                |
 |                        __ __  ____ |__|/  |_  ____                       |
 |                       |  |  \/    \|  \   __\/    \                      |
 |                       |  |  /   |  \  ||  | |   |  \                     |
 |                       |____/|___|  /__||__| |___|  /                     |
 |                       .__  __         .__                                |
 |                       |__|/  |______  |  | ___.__.                       |
 |                       |  \   __\__  \ |  |<   |  |                       |
 |                       |  ||  |  / __ \|  |_\___  |                       |
 |                       |__||__| (____  /____/ ____|                       |
 |                                     \/     \/                            |
 |                                                                          |
 |                                                                .         |
 |                                                                          |  
 |      Magnago Valerio                                                     |
 |      Bevilacqua Paolo                                                    |
 |      Dipartimento di Ingegneria e scenza dell'informazione               |
 |      Universita` degli Studi di Trento                                   |
 |      email: valerio.magnago@unitn.it                                     |
 |      email: paolo.bevilacqua@unitn.it                                    |
 |                                                                          |
\*--------------------------------------------------------------------------*/

#include <ros/ros.h>


#include <pthread.h>
#include <signal.h>
#include <mutex>

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

extern "C"
{
  #include "lego/client_bt.h"
  #include "lego/client_utils.h"
  #include "lego/pkt_serialization.h"
}

ros::Time msg_time;
struct cmd_pkt cmd = { 0, 0 };
int brick_socket = 0;



void  cmd_clbk(geometry_msgs::TwistConstPtr twist);
void  send_speed(const ros::TimerEvent&);
void* recv_thread_func(void* arg);

std::string pub_twist_topic_name = "/sensor/twist";
std::string cmd_sub_topic_name   = "/control/cmd_vel";
ros::Publisher  twist_pub;
ros::Subscriber cmd_sub;

int main(int argc, char **argv) {
  ros::init(argc, argv, "hardware_interface");
  ros::NodeHandle nh("~");
  
  char* arguments[2];
  int n_arg = 2;

    // parse mac address
  client_options options;
  if (opts_parse_config(&options, argc, argv) == -1)
  {
  	ROS_ERROR("Failed to parse the input");
    return -1;
  }

  ros::Timer timer = nh.createTimer(ros::Duration(0.1), send_speed);

  while (brick_bt_connect_device(&brick_socket, options.mac) < 0 )
  {
    ROS_WARN("Failed to open connection");
    if (!ros::ok()) return 0;
    usleep(1000*1000);
  }

  twist_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(pub_twist_topic_name, 1, false);
  cmd_sub   = nh.subscribe(cmd_sub_topic_name, 1 , &cmd_clbk, ros::TransportHints().tcpNoDelay());

  // run thread comm_thread
  pthread_t recv_thread;  
  pthread_create(&recv_thread, NULL, recv_thread_func, NULL);

  ros::spin();
  
  return 0;
}

void  send_speed(const ros::TimerEvent&){
  if((ros::Time::now() - msg_time).toSec() > 0.5){
  	cmd.v     = 0;
  	cmd.omega = 0;
  }

  uint8_t buffer[PKT_BUFFER_SIZE];
  size_t size;
  fill_cmd_msg(&cmd, buffer, &size);
  if (brick_bt_send(brick_socket, buffer, size) < 0)
  {
    fprintf(stderr, "Failed to send data\n");
  }
}

void  cmd_clbk(geometry_msgs::TwistConstPtr twist){  
  msg_time = ros::Time::now();
  cmd.v     = twist->linear.x;
  cmd.omega = twist->angular.z;
}

void* recv_thread_func(void* arg)
{
  uint8_t buffer[PKT_BUFFER_SIZE];
  struct status_pkt status;
  int res;

  double delta_t = std::numeric_limits<double>::max();
  int   read    = 0;


  while (ros::ok())
  {
    if ((res = brick_bt_recv(brick_socket, buffer+read, STATUS_PKT_SIZE-read, 100)) < 0)
    {
      if (res == -2)
      {
        ROS_INFO_THROTTLE(1, "Timeout\n"); 
      }
      else
      {
        ROS_WARN_THROTTLE(1, "Failed to receive data\n");
      }
    }
    else
    {	  
      read += res;
      if (read == STATUS_PKT_SIZE)
      {
        read = 0;
        parse_status_msg(buffer+PKT_HEADER_SIZE, &status);
                
        // Estimated speed   
	    geometry_msgs::TwistWithCovarianceStamped twist_msg;
	    twist_msg.header.frame_id   = "robot_footprint";
	    twist_msg.header.stamp  = ros::Time::now();
	    twist_msg.twist.twist.linear.x = status.v;
	    twist_msg.twist.twist.angular.z = status.omega;
	    twist_msg.twist.covariance[0]  = status.R11;
	    twist_msg.twist.covariance[35] = status.R22;
	    
	    twist_msg.twist.covariance[5]   = status.R12;
        twist_msg.twist.covariance[30]  = status.R12;
	    twist_pub.publish(twist_msg);
      }
    }
  }

  return 0;
}