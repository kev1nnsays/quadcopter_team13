// Basic math support functions
//
#include <stdlib.h>
#include <sys/time.h>
#include <math.h>

#define EXTERN extern 
#include "../include/quadcopter_main.h"

int64_t utime_now (void){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

int dec2oct(int dec){
  int ret = 0;
  if(dec != 0)
    ret = dec2oct(dec/8);
  return ret*10 + dec%8;
}

double minimize_angle(double theta){
  while(theta < -M_PI) theta = theta + 2*M_PI;
  while(theta >= M_PI) theta = theta - 2*M_PI;
  return theta;
}

int quaternion2euler(double q[4], double* euler){
  euler[0] = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));
  euler[1] = asin(2*(q[0]*q[2]-q[3]*q[1]));
  euler[2] = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
  return 0;
}

