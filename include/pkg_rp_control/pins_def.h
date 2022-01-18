#ifndef PINS_HH
#define PINS_HH

//PINS

constexpr int PWM_pin_1 = 1;   
constexpr int PWM_dir_1 = 27;   

constexpr int PWM_pin_2 = 23;   
constexpr int PWM_dir_2 = 25;

constexpr int bumper_pin = 3;

constexpr int EncR1 = 7;
constexpr int EncR2 = 0;
constexpr int EncL1 = 28;
constexpr int EncL2 = 29;

//MOTION PROFILE

constexpr int MAX_PWM_RANGE = 40;
constexpr double max_speed  = 15; // ?? from datasheet?
constexpr double perc_ramp  = 0.4;
constexpr double acc_delay  = 10;

//ERROR GAINS

constexpr double VIS_GAIN   = 0.01;
constexpr double ENC_GAIN   = 250;

#endif