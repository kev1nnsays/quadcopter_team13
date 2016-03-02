#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>

#define EXTERN extern
#define IMU_FREQ 50 // in Hz
#include "../include/quadcopter_main.h"
 
#define GYROX_BIAS 100
#define GYROY_BIAS 100
#define GYROZ_BIAS 100

#define INIT_RUNS 350
#define IMU_Q_SIZE 400

#define D2R 3.141592/180.0

wp_queue_t* acc_queue;
wp_queue_t* gyro_queue;
double biases[6];


void *run_imu(void * var)
{
  struct I2C_data gyro;
  struct I2C_data accel;
  int hz = IMU_FREQ;

  if(init_imu(&gyro,&accel) == -1) return NULL;
  imu_initialize_data();

  double gyro_data[3], accel_data[3];

  while(1){
    read_gyro(&gyro, gyro_data);
    read_accel(&accel, accel_data);
    gyro_data[0] = gyro_data[0]*D2R;
    gyro_data[1] = gyro_data[1]*D2R;
    gyro_data[2] = gyro_data[2]*D2R;

    pthread_mutex_lock(&imu_mutex);
    imudata.utime = (double)(utime_now())/1000000;
    imudata.gyro_x = gyro_data[0];
    imudata.gyro_y = gyro_data[1];
    imudata.gyro_z = gyro_data[2];
    imudata.accel_x = accel_data[0];
    imudata.accel_y = accel_data[1];
    imudata.accel_z = accel_data[2];
    if(0){
    printf("%ld,%lf,%lf,%lf,%lf,%lf,%lf\n",
      (long int) imudata.utime,imudata.gyro_x,imudata.gyro_y,imudata.gyro_z,
      imudata.accel_x,imudata.accel_y,imudata.accel_z);
    }
    //TODO: Median Filter
    if(imu_state->first_time){
      imu_state->prev_reading = imudata;
      imu_state->first_time = false;
    }
    else{
      float dt = imudata.utime - imu_state->prev_reading.utime ;
      //printf("IMU DT = %0.5lf\n",dt);
      // Bootstrap with pose from mocap
      // Set state pose to imu pose
      //XXX: Change to keep bootstrapping from mocap if possible..
      if(gyro_queue->end_idx == INIT_RUNS){
          memcpy(imu_state->pose, state->pose,8*sizeof(double));
          // Find means..
          for(int i=gyro_queue->start_idx;i<gyro_queue->end_idx;i++){
            biases[0]+=gyro_queue->buffer[3*i+0];
            biases[1]+=gyro_queue->buffer[3*i+1];
            biases[2]+=gyro_queue->buffer[3*i+2];
          }
          int npts = gyro_queue->end_idx-gyro_queue->start_idx+1;
          biases[0]/=npts;
          biases[1]/=npts;
          biases[2]/=npts;
          
          npts = acc_queue->end_idx-acc_queue->start_idx+1;
          for(int i=acc_queue->start_idx;i<acc_queue->end_idx;i++){
             biases[3]+=acc_queue->buffer[3*i+0];
            biases[4]+=acc_queue->buffer[3*i+1];
            biases[5]+=acc_queue->buffer[3*i+2];
          }
          biases[3]/=npts;
          biases[4]/=npts;
          biases[5]/=npts;
           printf("\nBiases (%lf,%lf,%lf,%lf,%lf,%lf)\n",
            biases[0],biases[1],biases[2],biases[3],biases[4],biases[5]);

        }
      if(0 && imu_grab_mocap && mocap_ready){
        puts("Resetting using MOCAP");
        memcpy(imu_state->pose, state->pose,8*sizeof(double));
        imu_grab_mocap = false;
      }
      if(gyro_queue->end_idx > INIT_RUNS){
          // Pose update
          for(int i=0;i<4;i++){
              imu_state->pose[i] += dt*imu_state->pose[4+i];
          }
          //imu_state->pose[2] = imu_state->pose[2] + dt*imu_state->pose[6];
          //imu_state->pose[3] = imu_state->pose[3] + dt*imu_state->pose[3];
          
          double acc_x = imudata.accel_x - biases[0];
          double acc_y = imudata.accel_y - biases[1];
          double acc_z = imudata.accel_z - biases[2];
          //double gyro_z = imudata.gyro_z - biases[5];
          
          //double acc_x = 0.0;
          //double acc_y = 0.0;
          //double acc_z = 0.0;
          double gyro_z = 0.0;
          // Update velocities
          double theta = imu_state->pose[3];
          imu_state->pose[4] += dt*(acc_x*cos(theta) - acc_y*sin(theta)) ;
          imu_state->pose[5] += dt*(acc_x*sin(theta) + acc_y*cos(theta)) ;
          imu_state->pose[6] += dt*acc_z;
          imu_state->pose[7] = gyro_z;
          // Publish 

          pose_t imu_pose_msg;
        imu_pose_msg.utime = 0.0f;
        imu_pose_msg.num_channels = 8;
        imu_pose_msg.channels = (float*) malloc(imu_pose_msg.num_channels*sizeof(float));
        float tmp;
        for(int i = 0; i < imu_pose_msg.num_channels; i++){
          tmp = imu_state->pose[i];
          imu_pose_msg.channels[i] = tmp;
        }

        pose_t imu_msg;
        imu_msg.utime = 0.0f;
        imu_msg.num_channels = 5;
        imu_msg.channels = (float*) malloc(imu_msg.num_channels*sizeof(float));
        imu_msg.channels[0] = acc_x;
        imu_msg.channels[1] = acc_y;
        imu_msg.channels[2] = acc_z;
        imu_msg.channels[3] = gyro_z;
        imu_msg.channels[4] = dt;
        

      // send lcm message to motors
       //puts("Publishing..");
     pose_t_publish((lcm_t *)lcm, "IMU_POSE", &imu_pose_msg);
      pose_t_publish((lcm_t *)lcm, "IMU", &imu_msg);
      free(imu_pose_msg.channels);
      free(imu_msg.channels);
       //puts("Published..");
    }

            imu_state->prev_reading = imudata;
  
    }

      wp_queue_push(gyro_queue,gyro_data);
      wp_queue_push(acc_queue,accel_data);
  //printf("IMU Q Size = %d\n",gyro_queue->end_idx);
    pthread_mutex_unlock(&imu_mutex);

    fprintf(imu_txt,"%ld,%lf,%lf,%lf,%lf,%lf,%lf\n",
      (long int) imudata.utime,imudata.gyro_x,imudata.gyro_y,imudata.gyro_z,
      imudata.accel_x,imudata.accel_y,imudata.accel_z);
    
    fflush(imu_txt);

    usleep(1000000/hz);

  }

  IMU_destroy(&gyro,&accel);
  fclose(imu_txt);
  return NULL;
}


int init_imu(struct I2C_data* gyro, struct I2C_data* accel){
  if(bbb_init() == -1){
    printf("Error initializing BBB.\n");
    return -1;
  }

  gyro->name = I2C_1;
  gyro->address = I2C_GYRO_ADDRESS;
  gyro->flags = O_RDWR;

  accel->name = I2C_1;
  accel->address = I2C_ACCEL_ADDRESS;
  accel->flags = O_RDWR;

  if(bbb_initI2C(gyro) == -1){
    printf("Error initializing I2C port for gyroscope.\n");
    return -1;
  }

  if(bbb_initI2C(accel) == -1){
    printf("Error initializing I2C port for accel/mag.\n");
    return -1;
  }

  byte buf[10];

  buf[0] = 0x20;
  buf[1] = 0x0F;
  bbb_writeI2C(gyro, buf, 2);

  buf[0] = 0x20;
  buf[1] = 0x67;
  bbb_writeI2C(accel, buf, 2);
  buf[0] = 0x24;
  buf[1] = 0xF0;
  bbb_writeI2C(accel, buf, 2);
  buf[0] = 0x26;
  buf[1] = 0x00;
  bbb_writeI2C(accel, buf, 2);

  return 0;
}

int imu_initialize_data()
{
  imu = (imu_data*) malloc(sizeof(struct imu_data));

  memset(imu->gyro_x_r,0,sizeof(imu->gyro_x_r));
  memset(imu->gyro_y_r,0,sizeof(imu->gyro_y_r));
  memset(imu->gyro_z_r,0,sizeof(imu->gyro_z_r));
  memset(imu->gyro_x_m,0,sizeof(imu->gyro_x_m));
  memset(imu->gyro_y_m,0,sizeof(imu->gyro_y_m));
  memset(imu->gyro_z_m,0,sizeof(imu->gyro_z_m));
  memset(imu->gyro_x,0,sizeof(imu->gyro_x));
  memset(imu->gyro_y,0,sizeof(imu->gyro_y));
  memset(imu->gyro_z,0,sizeof(imu->gyro_z));

  memset(imu->accel_x_r,0,sizeof(imu->accel_x_r));
  memset(imu->accel_y_r,0,sizeof(imu->accel_y_r));
  memset(imu->accel_z_r,0,sizeof(imu->accel_z_r));
  memset(imu->accel_x_m,0,sizeof(imu->accel_x_m));
  memset(imu->accel_y_m,0,sizeof(imu->accel_y_m));
  memset(imu->accel_z_m,0,sizeof(imu->accel_z_m));
  memset(imu->accel_x,0,sizeof(imu->accel_x));
  memset(imu->accel_y,0,sizeof(imu->accel_y));
  memset(imu->accel_z,0,sizeof(imu->accel_z));

  for(int i=0;i<6;i++){
    biases[i] = 0.0;
  }

  imu_state = (imu_state_t*) calloc(1,sizeof(imu_state_t));
  imu_state->first_time = true;
  imu_state->ready = false;

  gyro_queue = wp_queue_create(IMU_Q_SIZE,3);
  acc_queue = wp_queue_create(IMU_Q_SIZE,3);

  imu_txt = fopen("imu.txt","a");

  return 0;
}


void read_gyro(struct I2C_data *i2cd, double gyro[]){
  short tempint;
  byte buf, lobyte, hibyte;  // Used to store I2C data
  
  buf = 0x28; // X gyro, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x29; // X gyro, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;

  // GYROX_BIAS is the zero bias/offset - please adjust to make tempint close to zero for your x-gyro
  // printf("gyro x=%hd\n", tempint+GYROX_BIAS); // With your bias this should be near zero
  gyro[0] = 0.00875*(tempint+GYROX_BIAS);  // 110 is the zero bias/offset - please adjust
  
  buf = 0x2A; // Y gyro, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x2B; // Y gyro, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;

  // GYROY_BIAS is the zero bias/offset - please adjust to make tempint close to zero for your y-gyro
  // printf("gyro y=%hd\n", tempint + GYROY_BIAS); // With your bias this should be near zero
  gyro[1] = 0.00875*(tempint + GYROY_BIAS);  
  
  buf = 0x2C; // Z gyro, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x2D; // Z gyro, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = ((short) hibyte << 8) | lobyte;

  // GYROZ_BIAS is the zero bias/offset - please adjust to make tempint close to zero for your y-gyro
  // printf("gyro z=%hd\n", tempint + GYROZ_BIAS); // With your bias this should be near zero
  gyro[2] = 0.00875*(tempint + GYROZ_BIAS);

  return;
}


void read_accel(struct I2C_data *i2cd, double accel[]){
  short tempint;
  byte buf, lobyte, hibyte;  // Used to store I2C data
  
  buf = 0x28; // X accel, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x29; // X accel, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  accel[0] = 0.000061*tempint;
  
  buf = 0x2A; // Y accel, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x2B; // Y accel, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  accel[1] = 0.000061*tempint;
  
  buf = 0x2C; // Z accel, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x2D; // Z accel, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  accel[2] = 0.000061*tempint; 

  return;
}


void IMU_destroy(struct I2C_data *i2cd_gyro,struct I2C_data *i2cd_accelmag){
  bbb_deinitI2C(i2cd_gyro);
  bbb_deinitI2C(i2cd_accelmag);
  wp_queue_destroy(acc_queue);
  wp_queue_destroy(gyro_queue);
}

