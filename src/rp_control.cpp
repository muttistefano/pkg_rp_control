#include<pkg_rp_control/rp_control.h>
#include<pkg_rp_control/pins_def.h>
#include <signal.h>

std::string GetEnv( const std::string & var ) {
     const char * val = std::getenv( var.c_str() );
     if ( val == nullptr ) { 
         return "rp";
     }
     else {
         return val;
     }
}

platform_run::platform_run(ros::NodeHandle* nh_in, std::string host_name)
{
    _nh = *nh_in;
    _host_name = host_name;

    MovePlat  = _nh.advertiseService("MovePlat_" + _host_name, &platform_run::MovePlatSrvCall, this);
    _pub_conf = _nh.advertise<std_msgs::Int8>("Plat_conf_"+ _host_name, 1, true);

    this->pins_setup();

    std::thread t1(&platform_run::safety_task, this);
    t1.detach();

    std::thread t2(&platform_run::odometry, this);
    t2.detach();

    ros::Duration(1).sleep();

    std_msgs::Int8 conf_msg = std_msgs::Int8();
    conf_msg.data = 0;
    _pub_conf.publish(conf_msg);

    ROS_INFO("rp_control started \n");

}



bool platform_run::MovePlatSrvCall(pkg_rp_control::MovePlatSrv::Request  &req,
                              pkg_rp_control::MovePlatSrv::Response &res)
{
    //TODO
    std::cout << "Moving plat: " << _host_name << " to pos : " << int(req.position_command) << "\n";
    ros::Duration(2).sleep();
    res.position_final = req.position_command;
    res.success = true;
    std_msgs::Int8 conf_msg = std_msgs::Int8();
    conf_msg.data = res.position_final;
    _pub_conf.publish(conf_msg);
    return true;
}

void platform_run::safety_task()
{
  //TODO if there will be safety, ensure the button is pressed for a couple of cycles
  while (ros::ok())
  {
     this->_safe = 1;//digitalRead(bumper_pin);
     this->_rate.sleep();
  }
}

void my_handler(int s){
  printf("Caught signal %d\n",s);
  softPwmWrite(PWM_pin_1,0);
  softPwmWrite(PWM_pin_2,0);
  softPwmStop   (PWM_pin_1) ;
  softPwmStop   (PWM_pin_2) ;
  ros::shutdown();

}

void platform_run::pins_setup()
{
  
  softPwmCreate (PWM_pin_1, 0, MAX_PWM_RANGE) ;
  softPwmCreate (PWM_pin_2, 0, MAX_PWM_RANGE) ;
  pinMode (PWM_dir_1, OUTPUT) ;
  pinMode (PWM_dir_2, OUTPUT) ;

  pinMode (bumper_pin, INPUT) ;
  pullUpDnControl(bumper_pin,PUD_UP);

  pinMode (EncR1,INPUT);
  pinMode (EncR2,INPUT);
  pinMode (EncL1,INPUT);
  pinMode (EncL2,INPUT);

  if (wiringPiISR (EncR1, INT_EDGE_RISING, &interruptR1) < 0)
  {
    ROS_DEBUG("Interrupt setup error");
    return ;
  }

  if (wiringPiISR (EncR2, INT_EDGE_RISING, &interruptR2) < 0)
  {
    ROS_DEBUG("Interrupt setup error"); 
  }

  if (wiringPiISR (EncL1, INT_EDGE_RISING, &interruptL1) < 0)
  {
    ROS_DEBUG("Interrupt setup error");
    return ;
  }

  if (wiringPiISR (EncL2, INT_EDGE_RISING, &interruptL2) < 0)
  {
    ROS_DEBUG("Interrupt setup error");
    return ;
  }

  ROS_DEBUG("Pin setup finished");

}

void platform_run::reset_odom()
{
  Counter1_ = 0;
  Counter2_ = 0;
  Counter3_ = 0;
  Counter4_ = 0;
  posR1_    = 0.0;
  posR2_    = 0.0;
  posL1_    = 0.0;
  posL2_    = 0.0;
  this->curr_pos_ = 0.0;
}

void platform_run::odometry()
{
  // integrare angolo
  
  while(ros::ok())
  {
    this->enc_diff_   = (posR1_ + posR2_) - (posL1_ + posL2_);
    this->curr_pos_   = (posR1_ + posR2_ + posL1_ + posL2_)/4.0;
    this->_rate.sleep();
  }
}

int  platform_run::get_nom_speed(double movement_length_)
{
  int vel_ret = 0;

  if(this->curr_pos_ < (perc_ramp * movement_length_))
  {
    vel_ret = int( this->curr_pos_ * (max_speed/(perc_ramp * movement_length_))) + 2;
    return vel_ret;
  }
  if(this->curr_pos_ > (movement_length_ - (perc_ramp * movement_length_)))
  {
    vel_ret = int( (movement_length_ - this->curr_pos_) * (max_speed/(perc_ramp * movement_length_))) + 2;
    return vel_ret;
  }
  if(this->curr_pos_ < movement_length_ )
  {
    return int(max_speed);
  }
  std::cout << "EEERRRROOORORORORORROROROOR";
}

void platform_run::move(int forw,double movement_length_)
{
  
  if (forw == 0)
  {
    digitalWrite(PWM_dir_1,HIGH);
    digitalWrite(PWM_dir_2,HIGH);
  }
  else
  {
    digitalWrite(PWM_dir_1,LOW);
    digitalWrite(PWM_dir_2,LOW);
  }
  
  int corr      = 1;//forw ? 1:-1;
  int intensity = 0;
	
  
  while(this->curr_pos_ < movement_length_)
  {
    intensity = get_nom_speed(movement_length_);
    softPwmWrite(PWM_pin_1, intensity * this->_safe) ;	
    softPwmWrite(PWM_pin_2, intensity * this->_safe) ;	

    delay (acc_delay) ;
  }

  softPwmWrite (PWM_pin_1, 0) ;	
  softPwmWrite (PWM_pin_2, 0) ;	
  reset_odom();

}

int main(int argc, char **argv)
{      

//   struct sched_param param;
//           pthread_attr_t attr;
//           pthread_t thread;
//           int ret;
  
  /* Lock memory */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
          printf("mlockall failed: %m\n");
          exit(-2);
  }

//   /* Initialize pthread attributes (default values) */
//   ret = pthread_attr_init(&attr);
//   if (ret) {
//           printf("init pthread attributes failed\n");
//           exit(-2);
//   }

//   /* Set a specific stack size  */
//   ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
//   if (ret) {
//       printf("pthread setstacksize failed\n");
//       exit(-2);
//   }

//   /* Set scheduler policy and priority of pthread */
//   ret = pthread_attr_setschedpolicy(&attr, SCHED_RR);
//   if (ret) {
//           printf("pthread setschedpolicy failed\n");
//         exit(-2);
//   }

//   param.sched_priority = 80;
//   ret = pthread_attr_setschedparam(&attr, &param);
//   if (ret) {
//           printf("pthread setschedparam failed\n");
//           exit(-2);
//   }
//   /* Use scheduling parameters of attr */
//   ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
//   if (ret) {
//           printf("pthread setinheritsched failed\n");
//           exit(-2);
//   }

    if (wiringPiSetup () == -1)
    exit (1) ;


    char hostname[1024];
    gethostname(hostname, 1024);

    ros::init(argc, argv, "rp_control_" + std::string(hostname), ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");

    struct sigaction sigIntHandler;

    // sigIntHandler.sa_handler = my_handler;
    // sigemptyset(&sigIntHandler.sa_mask);
    // sigIntHandler.sa_flags = 0;

    // sigaction(SIGINT, &sigIntHandler, NULL);
    signal(SIGINT, my_handler);

    platform_run plat = platform_run(&nh,std::string(hostname));


    ros::AsyncSpinner spinner(4); 
    spinner.start();

    ros::waitForShutdown();


}


//TODO ignora fine coras in partenza