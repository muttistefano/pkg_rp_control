#include <wiringPi.h>
#include <softPwm.h>
#include "ros/ros.h"
#include <ros/console.h>
#include <pkg_rp_msgs/MovePlatSrv.h>

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
#include <std_msgs/Float64.h>
#include <fstream>

#include <limits.h>
#include <pthread.h>
#include <sys/mman.h>

#ifndef MOVIMENTO_HH
#define MOVIMENTO_HH

//Encoders data
std::atomic<double> posR1_(0.0);
std::atomic<double> posR2_(0.0);
std::atomic<double> posL1_(0.0);
std::atomic<double> posL2_(0.0);

std::atomic<int> Counter1_(0.0) ;
std::atomic<int> Counter2_(0.0) ;
std::atomic<int> Counter3_(0.0) ;
std::atomic<int> Counter4_(0.0) ;

class platform_run
{
public:
  platform_run(ros::NodeHandle *nh_, std::string host_name);
private:
    // pos data
    double curr_pos_;
    double enc_diff_ = 0.0;


    
    ros::Rate _rate = ros::Rate(150);

    ros::Subscriber sub_odom_;
    ros::ServiceServer MovePlat;
    ros::NodeHandle _nh;
    ros::Publisher _pub_conf;
    ros::Publisher _pub_odom;

    std::string _host_name;
    int _actual_configuration = 0;

    bool MovePlatSrvCall(pkg_rp_msgs::MovePlatSrv::Request  &req,
                         pkg_rp_msgs::MovePlatSrv::Response &res);

    void pins_setup();

    void odometry();
    void fine_corsa_1();
    void fine_corsa_2();

    void move(int forw,double movement_length_, double timeout);
    void reset_odom();
    int  get_nom_speed(double movement_length_,int min_over);
};

void interruptR1()
{
  Counter1_++;
  posR1_ = (static_cast<double>(Counter1_.load())/520.0) * 2.0 * M_PI * RAD_W;
}

void interruptR2()
{
  Counter2_++ ;
  posR2_ = (static_cast<double>(Counter2_.load())/520.0) * 2.0 * M_PI * RAD_W;
}

void interruptL1()
{
  Counter3_++ ;
  posL1_ = (static_cast<double>(Counter3_.load())/520.0) * 2.0 * M_PI * RAD_W;
}

void interruptL2()
{
  Counter4_++ ;
  posL2_ = (static_cast<double>(Counter4_.load())/520.0) * 2.0 * M_PI * RAD_W;
}

#endif

