#ifndef PINS_HH
#define PINS_HH

//PINS


constexpr int PWM_pin_1 = 27;   
constexpr int PWM_dir_1 = 28;   

constexpr int PWM_pin_2 = 24;   
constexpr int PWM_dir_2 = 25;

constexpr int bumper_pin_1 = 21;
constexpr int bumper_pin_2 = 23;

constexpr int EncR1 = 22;
constexpr int EncR2 = 26;
constexpr int EncL1 = 3;
constexpr int EncL2 = 4;

//MOTION PROFILE

constexpr double RAD_W      = 0.06;
constexpr int MAX_PWM_RANGE = 80;
constexpr int MIN_PWM_RANGE = 5;
constexpr int MAX_PWM_SPEED = 25;
constexpr double perc_ramp  = 0.3;

constexpr int finecorsa_int = 3;


#endif