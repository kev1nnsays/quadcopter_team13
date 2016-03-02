#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

int64_t utime_now (void);
int dec2oct(int dec);
double minimize_angle(double theta);
int quaternion2euler(double q[4], double* euler);

#endif
