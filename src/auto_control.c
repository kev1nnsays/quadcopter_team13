//
// auto_control:  Function to generate autonomous control PWM outputs.
//
#define EXTERN extern
#include "../include/quadcopter_main.h"

#define x_ref_dot_DownThres 0.2
#define x_ref_dot_ZeroThres 0.1

// Define outer loop controller
// PWM signal limits and neutral (baseline) settings

// THRUST
#define thrust_PWM_up 1575 // Upper saturation PWM limit.
#define thrust_PWM_base 1500 // Zero z_vela PWM base value.
#define thrust_PWM_down 1425 // Lower saturation PWM limit.

#define thrust_kp 0.0
#define thrust_ki 0.0
#define thrust_kd 0.0
#define thrust_isat 0.0

// ROLL
#define roll_PWM_left 1620  // Left saturation PWM limit.
#define roll_PWM_base 1500  // Zero roll_dot PWM base value.
#define roll_PWM_right 1380 //Right saturation PWM limit.

#define roll_kp 0.0
#define roll_ki 0.0
#define roll_kd 0.0
#define roll_isat
// PITCH
#define pitch_PWM_forward 1620  // Forward direction saturation PWM limit.
#define pitch_PWM_base 1500 // Zero pitch_dot PWM base value.
#define pitch_PWM_backward 1380 // Backward direction saturation PWM limit.

#define pitch_kp 0.0
#define pitch_ki 0.0
#define pitch_kd 0.0
#define pitch_isat 0.0
// YAW
#define yaw_PWM_ccw 1575 // Counter-Clockwise saturation PWM limit (ccw = yaw left).
#define yaw_PWM_base 1500 // Zero yaw_dot PWM base value.
#define yaw_PWM_cw 1425 // Clockwise saturation PWM limit (cw = yaw right).

#define yaw_kp 0.0
#define yaw_ki 0.0
#define yaw_kd 0.0
#define yaw_isat 0.0


// PID - Gives thrust to maintain x,xdot
float PID(float x,float x_dot,float x_ref,float x_ref_dot,
          float err_int, float kp,float ki,float kd,float trim)
{
  float e = x_ref - x;
  float e_dot = x_ref_dot - x_dot;
  float pterm = kp*e;
  float iterm = ki*err_int;
  float dterm = kd*e_dot;
  float sum = pterm+iterm+dterm+trim;
  return sum;
}

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

float PID_control(pid_state_t* pid, float x, float x_dot, float x_target, float x_ref_dot_init)
{
  double time_now = ((double)utime_now())/1000000;
  float dt = 0.0;
  float x_ref_dot;
  if(pid->first_time){
    pid->first_time = false;
    pid->prev_time = time_now;
    return pid->trim;
  }
  if(!pid->first_time){
     dt = time_now - pid->prev_time;
  }
  if(fabs(x_target - pid->x_ref) < 0.03){
    pid->x_ref = x_target;
  }
  else{
      pid->x_ref += x_ref_dot_init*dt;
  }
  if (fabs(x-x_target) < x_ref_dot_ZeroThres) {
      x_ref_dot = 0.0;
  }

  else  if (fabs(x-x_target) < x_ref_dot_DownThres) {
      x_ref_dot  = x_ref_dot_init - x_ref_dot_init*fabs(x-x_ref_dot_ZeroThres)/
          (x_ref_dot_DownThres - x_ref_dot_ZeroThres);
  }
  else {
      x_ref_dot = x_ref_dot_init;
  }

  float error_dot = x_ref_dot - x_dot;
  pid->x_ref_dot = x_ref_dot;
  //printf("Xref = %f\t DT = %f\n",pid->x_ref,dt);

  float error = pid->x_ref - x;
  pid->iterm += pid->ki*error;
  pid->iterm = clip(pid->iterm,pid->isat);

  float output = pid->iterm + pid->kp*error + pid->kd*error_dot ;
  output = clip(output,pid->outputsat) + pid->trim;

  pid->prev_time = time_now;
  pid->error = error;
  pid->error_dot = error_dot;
  pid->output = output;
  return output;
}
// Outer loop controller to generate PWM signals for the Naza-M autopilot
void auto_control(float *pose, float *set_points, int16_t* channels_ptr)
{

  channels_ptr[0] = PID_control(pid_thrust,pose[2],pose[6],set_points[2],set_points[6]); // Alt
  channels_ptr[1] = PID_control(pid_roll,pose[1],pose[5],set_points[1],set_points[5]); // y
  channels_ptr[2] = PID_control(pid_pitch,pose[0],pose[4],set_points[0],set_points[4]); // x
  channels_ptr[3] = PID_control(pid_yaw,pose[3],pose[7],set_points[3],set_points[7]); // yaw


  return;
}
