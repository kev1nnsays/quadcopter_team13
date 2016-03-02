#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>


#include <lcm/lcm.h>
#include "../include/lcmtypes/lcmtypes_c/channels_t.h"
#include "../include/quadcopter_struct.h"
#include "../include/bbblib/bbb.h"

// Primary threads
#include "../include/run_imu.h"
#include "../include/run_motion_capture.h"
#include "../include/util.h"

int main() {

  // Open file to capture command RX, TX, and pertinent guidance data
  pthread_mutex_t imu_mutex; 

  // Initialize the data mutexes
  pthread_mutex_init(&imu_mutex, NULL);

  // Start the threads
  pthread_t imu_thread;

  // UNCOMMENT TO USE IMU
  pthread_create(&imu_thread, NULL, run_imu, NULL);
 
  // UNCOMMENT TO USE IMU
  pthread_join(imu_thread, NULL);
 
  return 0;
}