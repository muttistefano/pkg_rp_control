#include <wiringPi.h>
#include <softPwm.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <pkg_rp_control/MovePlatSrv.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <signal.h>
#include <chrono>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int8.h>
#include <fstream>


#ifndef MOVIMENTO_HH
#define MOVIMENTO_HH

//Encoders data
double posR1_ = 0.0;
double posR2_ = 0.0;
double posL1_ = 0.0;
double posL2_ = 0.0;

double Counter1_ = 0 ;
double Counter2_ = 0 ;
double Counter3_ = 0 ;
double Counter4_ = 0 ;

class platform_run
{
public:
  platform_run(ros::NodeHandle *nh_, std::string host_name);
private:
    // pos data
    double curr_pos_;
    double enc_diff_ = 0.0;


    // safety data
    int _safe;
    
    ros::Rate _rate = ros::Rate(50);

    ros::Subscriber sub_odom_;
    ros::ServiceServer MovePlat;
    ros::NodeHandle _nh;
    ros::Publisher _pub_conf;

    std::string _host_name;
    int _actual_configuration = 0;

    bool MovePlatSrvCall(pkg_rp_control::MovePlatSrv::Request  &req,
                         pkg_rp_control::MovePlatSrv::Response &res);

    void pins_setup();

    void odometry();
    void safety_task();

    void move(int forw,double movement_length_);
    void reset_odom();
    int  get_nom_speed(double movement_length_);

};

void interruptR1()
{
  ++Counter1_ ;
  posR1_ = (Counter1_/520.0) * 2.0 * M_PI * 0.06;
}

void interruptR2()
{
  ++Counter2_ ;
  posR2_ = (Counter2_/520.0) * 2.0 * M_PI * 0.06;
}

void interruptL1()
{
  ++Counter3_ ;
  posL1_ = (Counter3_/520.0) * 2.0 * M_PI * 0.06;
}

void interruptL2()
{
  ++Counter4_ ;
  posL2_ = (Counter4_/520.0) * 2.0 * M_PI * 0.06;
}

#endif

