#include "PID.h"

//#include "STM32Hardware.h"
//#include "hardwareserial.h"
extern float pid_p1 ;
extern float pid_i1 ;
extern float pid_d1 ;

extern float pid_p2 ;
extern float pid_i2 ;
extern float pid_d2 ;

PID::PID(float min_val, float max_val, float kp, float ki, float kd)
{
  min_val_ = min_val;
  max_val_ = max_val;
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}


double PID::compute2(float setpoint, float measured_value)
{
  double error;
  double pid;

  //setpoint is constrained between min and max to prevent pid from having too much error
  error = setpoint - measured_value;
  integral_ += error;
  derivative_ = error - prev_error_;

  if(setpoint == 0 && error == 0){
    integral_ = 0;
  }

  pid = (pid_p2 * error) + (pid_i2 * integral_) + (pid_d2 * derivative_);
  prev_error_ = error;

  return constrain(pid, min_val_, max_val_);
}

int pid;

double PID::compute(float setpoint, float measured_value)
{
  double error;
  double pid_t;
  //setpoint is constrained between min and max to prevent pid from having too much error
  error = setpoint - measured_value;
  integral_ += error;
  derivative_ = error - prev_error_;

  if(setpoint == 0 && error == 0){
    integral_ = 0;
  }
 
 // pid_t = (K_P * error) + (K_I * integral_) + (K_D * derivative_);
  pid_t = (pid_p1 * error) + (pid_i1 * integral_) + (pid_d1 * derivative_);
  pid = (int)pid_t;
  prev_error_ = error;
  return constrain(pid_t, min_val_, max_val_);
}

void PID::updateConstants(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
