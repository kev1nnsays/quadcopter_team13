#ifndef QUADCOPTER_GLOBALS
#define QUADCOPTER_GLOBALS

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

//Use this in your code:  #define EXTERN extern

#include <lcm/lcm.h>
#include "../include/lcmtypes/lcmtypes_c/channels_t.h"
#include "../include/lcmtypes/lcmtypes_c/fsm_state_t.h"
#include "../include/lcmtypes/lcmtypes_c/pose_t.h"
#include "../include/lcmtypes/lcmtypes_c/waypoint_trigger_t.h"
#include "../include/lcmtypes/lcmtypes_c/pid_status_t.h"
#include "../include/quadcopter_struct.h"
#include "../include/bbblib/bbb.h"

// Primary threads
#include "../include/run_imu.h"
#include "../include/run_motion_capture.h"
#include "../include/util.h"

#include "wp_queue.h"
// Custom

EXTERN bool watchdog_panic;
EXTERN bool first_time_set_pt;
EXTERN bool imu_grab_mocap;
EXTERN bool mocap_ready;

EXTERN bool manual_perch_mode;

/* Global Variables with mutexes for sharing */
EXTERN lcm_t* lcm;
EXTERN imu_t imudata;
EXTERN struct motion_capture_obs mcap_obs[2];
EXTERN state_t *state;
EXTERN imu_state_t *imu_state;
EXTERN wp_queue_t* waypoint_queue;
EXTERN DynamBus bus;
EXTERN int auto_state;

EXTERN quad_state_t fsm_quad_state;



EXTERN float target_pt1[3];
EXTERN float target_pt2[3];
EXTERN float target_z;
EXTERN float target_y;
EXTERN float target_x;
EXTERN float safe_mid_pt[3];
EXTERN float my_set_points[8];

EXTERN double setpt_reached_time;
EXTERN int setpt_stable;

EXTERN pid_state_t* pid_thrust;
EXTERN pid_state_t* pid_roll;
EXTERN pid_state_t* pid_pitch;
EXTERN pid_state_t* pid_yaw;

/* Mutexes */
EXTERN pthread_mutex_t imu_mutex;
EXTERN pthread_mutex_t mcap_mutex;
EXTERN pthread_mutex_t state_mutex;
EXTERN pthread_mutex_t dynamixel_mutex;
EXTERN pthread_mutex_t waypoints_mutex;

/* Global variables that are not used in multiple threads (no mutex use) */
EXTERN FILE *mcap_txt, *block_txt, *imu_txt, *pid_txt, *fsm_txt; // Output data files

EXTERN int imu_mode, mcap_mode;
EXTERN struct imu_data *imu;
EXTERN DynamSetup dynam;
EXTERN int commandGripper;
EXTERN float time_turned_off_cmd;


// Variables and flags for FSM
EXTERN bool quad_cmd_perch; // Quagrotor tells gripper to grasp
EXTERN int dynam_perch_signal;  // Dynamixel signals the quadrotor whether it has grasped something
EXTERN bool quad_open_claw; // Quad tells gripper to open claw
EXTERN bool dynam_claw_open; // Gripper tells the quad that claw is open


EXTERN bool turn_off_propellers;
EXTERN bool take_off_flag;
EXTERN double time_turned_off;


/* Function declarations needed for multiple files */
void channels_handler(const lcm_recv_buf_t *rbuf, const char *channel,
		      const channels_t *msg, void *userdata);
void wp_trigger_handler(const lcm_recv_buf_t *rbuf, const char *channel,
		      const waypoint_trigger_t *msg, void *userdata);
void auto_control(float *pose, float *set_points, int16_t *channels_ptr);


// Top-level thread function declarations
void *lcm_thread_loop(void *);
void *key_in_loop(void *);
void *processing_loop(void *);
void *run_imu(void *);
void *run_motion_capture(void *);

// PID stuff
void PID_soft_reset(pid_state_t* pid);
void PID_reset(pid_state_t* pid);
void PID_print(pid_state_t* pid);
void PID_reset_all(void);

// LCM Stuff
void publish_state(void);
void publish_set_point(void);
void spoof_transmitter(void);
void publish_pid_state(pid_state_t* pid);
// Q stuff
bool grab_set_point();

// Setting points
EXTERN float perch_pose[8];
EXTERN float safe_pose[8];
EXTERN char quad_state_name[128];
// Check if current pose is safe
bool check_safe(void);
// Set pose as setpoint
void set_curr_pose(void);
// Set pose as setpoint
void set_safe_point(void);
// Set pose as setpoint
void set_perch_point(void);
// Set current point as perch and a point above it as safe
void  set_curr_pt_perch(void);

bool check_alignment_manual(void);
// Print stuff
void floats_print(float* wp);

void doubles_print(double* wp);

//////////////////////////////////////////
///  Geofence stuff for testing
//////////////////////////////////////
#define NUM_SAMPLES_MED_ALT 10
#define NUM_SAMPLES_AVG_ALT 20

EXTERN float KP_thrust, KI_thrust, KD_thrust;
EXTERN float KP_pitch, KI_pitch, KD_pitch;
EXTERN float KP_roll, KI_roll, KD_roll;
EXTERN float KP_yaw, KI_yaw, KD_yaw;

EXTERN double diff_z[NUM_SAMPLES_MED_ALT], diff_z_med[NUM_SAMPLES_AVG_ALT];
EXTERN double start[2];
EXTERN double ang_buf;
EXTERN double fence_penalty_length;

// max desired velocity for fence corrections
EXTERN float max_vel, max_step;

EXTERN double alt_low_fence, alt_high_fence;
EXTERN double x_pos_fence, x_neg_fence, y_pos_fence, y_neg_fence;
EXTERN double alt_prev;


//////////////////////////////////////////////////////////////////////////////
// Geofence function
int update_set_points(double* pose, float* set_points, int first_time);

#ifdef __cplusplus
}
#endif

#endif
