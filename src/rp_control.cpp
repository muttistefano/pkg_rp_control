#include<pkg_rp_control/pins_def.h>
#include<pkg_rp_control/rp_control.h>
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

    // std::thread t1(&platform_run::safety_task, this);
    // t1.detach();

    std::thread t2(&platform_run::odometry, this);
    t2.detach();

    ros::Duration(1).sleep();

    std_msgs::Int8 conf_msg = std_msgs::Int8();
    conf_msg.data = 0;
    _pub_conf.publish(conf_msg);

    ROS_INFO("rp_control started \n");

}



bool platform_run::MovePlatSrvCall(pkg_rp_msgs::MovePlatSrv::Request  &req,
                              pkg_rp_msgs::MovePlatSrv::Response &res)
{
    //TODO prendi distanze
    std::cout << "Moving plat: " << _host_name << " to pos : " << int(req.position_command) << "\n";
    if (req.position_command == 0)
    {
      this->move(req.position_command ,1.95,10);
    }
    else if (req.position_command == 1)
    {
      this->move(req.position_command ,1.95,10);
    }
    else if (req.position_command == 100)
    {
      this->recover(req.position_command ,1,10);
    }
    else
    {
      return false;
    }
    
    if (req.position_command == 100)
    {
      res.position_final = 0;
      res.success = true;
      std_msgs::Int8 conf_msg = std_msgs::Int8();
      conf_msg.data = res.position_final;
      _pub_conf.publish(conf_msg);
    }
    else
    {
      res.position_final = req.position_command;
      res.success = true;
      std_msgs::Int8 conf_msg = std_msgs::Int8();
      conf_msg.data = res.position_final;
      _pub_conf.publish(conf_msg);
    }


    return true;
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

  pinMode (bumper_pin_1, INPUT) ;
  pinMode (bumper_pin_2, INPUT) ;
  pullUpDnControl(bumper_pin_1,PUD_UP);
  pullUpDnControl(bumper_pin_2,PUD_UP);

  pinMode (EncR1,INPUT);
  pinMode (EncR2,INPUT);
  pinMode (EncL1,INPUT);
  pinMode (EncL2,INPUT);
  pullUpDnControl(EncR1,PUD_UP);
  pullUpDnControl(EncR2,PUD_UP);
  pullUpDnControl(EncL1,PUD_UP);
  pullUpDnControl(EncL2,PUD_UP);

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
  ROS_INFO_STREAM(posL1_ << " " << posL2_ << " " << posR1_ << " " << posR2_ << "\n");
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
    
  while(ros::ok())
  {
    std::vector<double> posvec = {posR1_,posR2_,posL1_,posL2_};
    std::sort(posvec.begin(),posvec.end());
    this->curr_pos_   = (posvec[1] + posvec[2])/2.0;

    this->_rate.sleep();
  }
}

int  platform_run::get_nom_speed(double movement_length_,int min_over)
{
  int vel_ret = 0;

  if(this->curr_pos_ < (perc_ramp * movement_length_))
  {
    vel_ret = int( this->curr_pos_ * (MAX_PWM_SPEED/(perc_ramp * movement_length_)));
    return std::max(vel_ret,MIN_PWM_RANGE + min_over);
  }
  if(this->curr_pos_ > (movement_length_ - (perc_ramp * movement_length_)))
  {
    vel_ret = int( (movement_length_ - this->curr_pos_) * (MAX_PWM_SPEED/(perc_ramp * movement_length_)));
    return std::max(vel_ret,MIN_PWM_RANGE + min_over);
  }
  if(this->curr_pos_ < movement_length_ )
  {
    return int(MAX_PWM_SPEED);
  }
  std::cout << "EEERRRROOORORORORORROROROOR";
  return MIN_PWM_RANGE;
}

void platform_run::move(int forw,double movement_length_,double timeout)
{
  
  int finecorsa = 0;

  if (forw == 0)
  {
    digitalWrite(PWM_dir_1,LOW);
    digitalWrite(PWM_dir_2,HIGH);
  }
  else
  {
    digitalWrite(PWM_dir_1,HIGH);
    digitalWrite(PWM_dir_2,LOW);
  }
  
  int intensity  = 0;
	int min_over   = 0;
  double pos_wd  = 0;

  auto start = std::chrono::steady_clock::now();
  auto wd_init = std::chrono::steady_clock::now();

  while((this->curr_pos_ < (movement_length_ + 0.5)) && ros::ok())
  {
    
    finecorsa += (forw == 0) ? (digitalRead(bumper_pin_1) == 0) : (digitalRead(bumper_pin_2) == 0);
    if (finecorsa > finecorsa_int)
    {
      ROS_INFO_STREAM("finecorsa");
      break;
    }

    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() > 250)
    {
      start = std::chrono::steady_clock::now(); 
      if ((this->curr_pos_ - pos_wd) < 0.0015)
      {
        min_over = min_over + 1;
        // ROS_WARN_STREAM("override: " << min_over << "\n");
      }
      else
      {
        if(this->curr_pos_ > (movement_length_/2.0 ))
        {
          min_over = (min_over > 1) ? min_over - 1 : 0 ;
        }
      }
      pos_wd = this->curr_pos_;
    }

    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - wd_init).count() > 120000)
    {return;}
    
    intensity = this->get_nom_speed(movement_length_,min_over);
    // ROS_WARN_STREAM_THROTTLE(0.5,"override: " << intensity << "\n");
    softPwmWrite(PWM_pin_1, intensity ) ;	
    softPwmWrite(PWM_pin_2, intensity ) ;	
    this->_rate.sleep();
  }

  softPwmWrite (PWM_pin_1, 0) ;	
  softPwmWrite (PWM_pin_2, 0) ;	
  reset_odom();

}

void platform_run::recover(int forw,double movement_length_,double timeout)
{
  
  int finecorsa = 0;


  digitalWrite(PWM_dir_1,LOW);
  digitalWrite(PWM_dir_2,HIGH);

  
  int intensity  = 0;
	int min_over   = 0;
  double pos_wd  = 0;

  auto start = std::chrono::steady_clock::now();
  while((this->curr_pos_ < (movement_length_ )) && ros::ok())
  {
    
    finecorsa += (forw == 100) ? (digitalRead(bumper_pin_1) == 0) : (digitalRead(bumper_pin_2) == 0);
    if (finecorsa > finecorsa_int)
    {
      ROS_INFO_STREAM("finecorsa");
      break;
    }

    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() > 500)
    {
      start = std::chrono::steady_clock::now(); 
      if ((this->curr_pos_ - pos_wd) < 0.0025)
      {
        min_over = min_over + 1;
        ROS_WARN_STREAM("override: " << min_over << "\n");
      }
      else
      {
        min_over = (min_over > 1) ? min_over - 1 : 0 ;
      }
      pos_wd = this->curr_pos_;
    }

    intensity = MIN_PWM_RANGE + min_over;
    softPwmWrite(PWM_pin_1, intensity ) ;	
    softPwmWrite(PWM_pin_2, intensity ) ;	
    this->_rate.sleep();
  }

  softPwmWrite (PWM_pin_1, 0) ;	
  softPwmWrite (PWM_pin_2, 0) ;	
  reset_odom();

}

void *thread_func(void *data)
{

  char hostname[1024];
  gethostname(hostname, 1024);

  ros::NodeHandle nh("~");
  platform_run plat = platform_run(&nh,std::string(hostname));

  ros::AsyncSpinner spinner(4); 
  spinner.start();

  ros::waitForShutdown();
  return NULL;
}

int main(int argc, char **argv)
{      

  struct sched_param param;
  pthread_attr_t attr;
  pthread_t thread;
  int ret;
  
  /* Lock memory */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
          printf("mlockall failed: %m\n");
          exit(-2);
  }

  /* Initialize pthread attributes (default values) */
  ret = pthread_attr_init(&attr);
  if (ret) {
          printf("init pthread attributes failed\n");
          exit (1) ;
  }

  /* Set a specific stack size  */
  ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
  if (ret) {
      printf("pthread setstacksize failed\n");
      exit (1) ;
  }

  /* Set scheduler policy and priority of pthread */
  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret) {
          printf("pthread setschedpolicy failed\n");
          exit (1) ;
  }
  param.sched_priority = 80;
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret) {
          printf("pthread setschedparam failed\n");
          exit (1) ;
  }
  /* Use scheduling parameters of attr */
  ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (ret) {
          printf("pthread setinheritsched failed\n");
          exit (1) ;
  }



  if (wiringPiSetup () == -1)
  exit (1) ;


  char hostname[1024];
  gethostname(hostname, 1024);

  ros::init(argc, argv, "rp_control_" + std::string(hostname), ros::init_options::NoSigintHandler);
  // ros::NodeHandle nh("~");

  struct sigaction sigIntHandler;

  signal(SIGINT, my_handler);

  /* Create a pthread with specified attributes */
  ret = pthread_create(&thread, &attr, thread_func, NULL);
  if (ret) {
          printf("create pthread failed\n");
          exit (1) ;
  }

  /* Join the thread and wait until it is done */
  ret = pthread_join(thread, NULL);
  if (ret)
          printf("join pthread failed: %m\n");


}


