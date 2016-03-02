// PID_controller
#include <stdlib.h>
#include <stdio.h>

#define kp 0
#define ki 0
#define kd 0
#define isat 0

int flag_target = 0;
float iTerm = 0;

float clip(float in, float maxabs){
  if (in > maxabs)
  {
    in = maxabs;
  }
  if (in < -maxabs)
  {
    in = -maxabs;
  }
  return in;
}

float PID_control(float ki, float kp, float kd, float isat, float x_ref_dot, 
                  float dt, float trim, float x, float x_dot, float x_target, float outputSat)
{ 
  float x_ref = x_ref_dot*dt;
  float error_dot = x_ref_dot - x_dot;
  if (x >= x_target)
  {
    flag_target = 1;
  }
  if (flag_target == 1)
  {
    x_ref = x_target;
    trim = 0;
    error_dot = 0;
  }
  
  float error = x_ref - x; 
  iTerm += ki*error;
  iTerm = clip(iTerm,isat);
  
  float output = iTerm + kp*error + kd*error_dot + trim;
  output = clip(output,outputSat);
  
  return output;
}
