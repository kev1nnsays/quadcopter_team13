#ifndef MOTION_CAPTURE_H
#define MOTION_CAPTURE_H

#include <stdio.h>
#include <inttypes.h>

struct motion_capture_obs{
  double time;
  // position (x,y,z,roll,pitch,yaw)
  double pose[6];
};

struct optitrack_message {
  int ID;
  float x, y, z;
  float qx, qy, qz, qw;
};

// Top-level thread function that runs Optitrack's PacketClient.cpp code
// copying received data to our shared data structure with mutex use.
void *run_motion_capture(void *userdata);

#endif
