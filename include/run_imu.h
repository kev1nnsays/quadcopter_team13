#ifndef IMU_H
#define IMU_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>

#include "bbblib/bbb.h"
#include "util.h"

#define I2C_GYRO_ADDRESS 0xD6
#define I2C_ACCEL_ADDRESS 0x3A
#define NUM_SAMPLES_MED 5
#define NUM_SAMPLES_AVG 20

// Top level thread function
void *run_imu(void *);

/*
 * imu_data:
 * structure to hold recent measurements and filtered values from imu
*/
struct imu_data{
  // Raw values lists for 1st stage filtering
  double accel_x_r[NUM_SAMPLES_MED], 
    accel_y_r[NUM_SAMPLES_MED], 
    accel_z_r[NUM_SAMPLES_MED];
  double gyro_x_r[NUM_SAMPLES_MED], 
    gyro_y_r[NUM_SAMPLES_MED], 
    gyro_z_r[NUM_SAMPLES_MED];

  // Median filtered values lists for averaging
  double accel_x_m[NUM_SAMPLES_AVG], 
    accel_y_m[NUM_SAMPLES_AVG], 
    accel_z_m[NUM_SAMPLES_AVG];
  double gyro_x_m[NUM_SAMPLES_AVG], 
    gyro_y_m[NUM_SAMPLES_AVG], 
    gyro_z_m[NUM_SAMPLES_AVG];

  // Averaging filtered values lists
  double accel_x[NUM_SAMPLES_AVG], 
    accel_y[NUM_SAMPLES_AVG], 
    accel_z[NUM_SAMPLES_AVG];
  double gyro_x[NUM_SAMPLES_AVG], 
    gyro_y[NUM_SAMPLES_AVG], 
    gyro_z[NUM_SAMPLES_AVG];
};

// For internal use only
int init_imu(struct I2C_data* gyro, struct I2C_data* accel);
int imu_initialize_data();
void read_gyro(struct I2C_data *i2cd, double gyro[]);
void read_accel(struct I2C_data *i2cd, double accel[]);
void IMU_destroy(struct I2C_data *i2cd_gyro,struct I2C_data *i2cd_accelmag);

#endif
