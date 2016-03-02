//
// Top-level program for quadcopter
//
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>

#define EXTERN   // main() program definition
#define IN_FREQ 10
#define Z_SAFE 0.5
#include "../include/quadcopter_main.h"

/*  ########################################
    ###    QUADCOPTER MAIN FUNCTION      ###
    ########################################

    Code has several threads.
    Comment the ones you do not need.
    List below show the threads you NEED to run it.

    ### FOR THE QUADROTOR TO FLY ###
    -> lcm_thread_loop
    -> processing_loop
    -> run_motion_capture
    -> (if you use servos) run_dynamixel_comm
    -> (if you use imu)    run_imu

    ### FOR TESTING MOTION CAPTURE ONLY ###
    -> run_motion_capture

    ### FOR TESTING SERVOS ONLY (NOT CONNECTED TO YOUR GRIPPER!) ###
    -> run_dynamixel_comm
    -> set_dynamixel
*/

int main() {

  // Open file to capture command RX, TX, and pertinent guidance data
  block_txt = fopen("block.txt","a");
  pid_txt = fopen("pid.txt","a");
  fsm_txt = fopen("fsm.txt","a");
  imu_grab_mocap = false;
  mocap_ready = false;

double wp[8] = {0.1,0.2,0.3,0.4,
                0.5,0.6,0.7,0.8};
  // Init q
  waypoint_queue = wp_queue_create(1000,8);
  wp_queue_push(waypoint_queue, wp);
  wp_queue_push(waypoint_queue, wp);
  wp_queue_pop(waypoint_queue);
  // Initialize the data mutexes
  pthread_mutex_init(&imu_mutex, NULL);
  pthread_mutex_init(&mcap_mutex, NULL);
  pthread_mutex_init(&state_mutex, NULL);
  pthread_mutex_init(&dynamixel_mutex, NULL);

  // Start the threads
  pthread_t lcm_thread;
  pthread_t processing_thread;
  pthread_t keyboard_thread;
  pthread_t imu_thread;
  pthread_t motion_capture_thread;
  pthread_t dynamixel_comm_thread;
  pthread_t dynamixel_set_thread;

  pthread_create(&lcm_thread, NULL, lcm_thread_loop, NULL);
  pthread_create(&processing_thread, NULL, processing_loop, NULL);
  pthread_create(&keyboard_thread, NULL, key_in_loop, NULL);
  pthread_create(&imu_thread, NULL, run_imu, NULL);
  pthread_create(&motion_capture_thread, NULL, run_motion_capture, NULL);

  pthread_create(&dynamixel_comm_thread, NULL, run_dynamixel_comm, NULL);
  sleep(2);
  pthread_create(&dynamixel_set_thread, NULL, set_dynamixel, NULL);

  // Join threads upon completetion
  pthread_join(lcm_thread, NULL);
  pthread_join(processing_thread, NULL);
  pthread_join(keyboard_thread, NULL);
  pthread_join(imu_thread, NULL);
  pthread_join(motion_capture_thread, NULL);
  pthread_join(dynamixel_comm_thread, NULL);
  pthread_join(dynamixel_set_thread, NULL);

  fclose(block_txt);
  fclose(pid_txt);
  return 0;
}

void floats_print(float* wp){
  printf("\n");
  printf("X = %0.5f\tY = %0.5f\tZ = %0.5f\tYaw = %0.5f\n",wp[0],wp[1],wp[2],wp[3]);
  printf("XD = %0.5f\tYD = %0.5f\tZD = %0.5f\tYawD = %0.5f\n",wp[4],wp[5],wp[6],wp[7]);
  printf("\n");
}

void doubles_print(double* wp){
  printf("\n");
  printf("X = %0.5lf\tY = %0.5lf\tZ = %0.5lf\tYaw = %0.5f\n",wp[0],wp[1],wp[2],wp[3]);
  printf("XD = %0.5lf\tYD = %0.5lf\tZD = %0.5lf\tYawD = %0.5f\n",wp[4],wp[5],wp[6],wp[7]);
  printf("\n");
}

void pose_print(float* wp){
  printf("\n");
  printf("X = %0.5f\tY = %0.5f\tZ = %0.5f\tYaw = %0.5f\n",wp[0],wp[1],wp[2],wp[3]);
  printf("XD = %0.5f\tYD = %0.5f\tZD = %0.5f\tYawD = %0.5f\n",wp[4],wp[5],wp[6],wp[7]);
  printf("\n");
}



void PID_reset(pid_state_t* pid){
  pid->first_time = true;
  pid->iterm = 0.0;
  pid->reached_target = false;
  pid->x_ref = state->pose[pid->track_id];
  // printf("PID %s RESET HARD..\n",pid->name);
}

void PID_soft_reset(pid_state_t* pid){
  pid->reached_target = true;
  // printf("PID %s RESET SOFT..\n",pid->name);
}


void IMU_print(imu_state_t* imu){
  puts("IMU Pose\n----------\n");
  wp_print(imu->pose,8);
}


void PID_print(pid_state_t* pid){
  printf("PID %s Status\n---------------\n",pid->name);
  printf(" Gains = (%0.5f,%0.5f,%0.5f)\n",pid->kp,pid->ki,pid->kd);
  printf(" Trim = %0.5f\t ISat = %0.5f \t OutSat = %0.5f\n",pid->trim,pid->isat,pid->outputsat);
  puts("");
  printf("ITerm = %0.5f\t Prev_Time = %0.5lf\t %s\t %s\n------------------\n\n",
        pid->iterm,pid->prev_time,
        pid->reached_target==1?"REACHED":"NOT REACHED",
        pid->first_time?"FIRST TIME":"NOT FIRST TIME");
}

void PID_reset_all(void){
    PID_reset(pid_thrust);
    PID_reset(pid_roll);
    PID_reset(pid_pitch);
    PID_reset(pid_yaw);

}


/*
  LCM processing (top-level loop) -- Only use to talk with BLOCKS
*/
void *lcm_thread_loop(void *data){
  printf("lcm running");
  lcm = lcm_create(NULL);
  channels_t_subscribe(lcm, "CHANNELS_1_RX", channels_handler, lcm);
  waypoint_trigger_t_subscribe(lcm, "WP_TRIGGER", wp_trigger_handler, lcm);
  while(1){
    lcm_handle(lcm);
    usleep(10);
  }
  lcm_destroy(lcm);
  return 0;
}


void print_state(){
    printf("===========\n");
    printf("STATE\n");
    printf("===========\n\n");

    printf(auto_state==1?"\nAUTO ON\n":"\nAUTO OFF\n");

    printf("POSE\n");
    doubles_print(state->pose);
    puts("");

    printf("SET POINT\n");
    floats_print(my_set_points);
    puts("");

    printf("===========\n");
    printf("WAYPOINT QUEUE\n");
    printf("===========\n\n");

    wp_queue_print(waypoint_queue);

}

void publish_state(void){
        pose_t pose_msg;
        pose_msg.utime =  ((double)utime_now())/1000000.0;
        pose_msg.num_channels = 8;
        pose_msg.channels = (float*) malloc(pose_msg.num_channels*sizeof(float));
        float tmp;

        for(int i = 0; i < pose_msg.num_channels; i++){
          tmp = state->pose[i];
          pose_msg.channels[i] = tmp;
        }
      pose_t_publish((lcm_t *)lcm, "POSE", &pose_msg);
      free(pose_msg.channels);
}
void publish_set_point(void){
        pose_t pose_msg;
        pose_msg.utime = 0.0f;
        pose_msg.num_channels = 8;
        pose_msg.channels = (float*) malloc(pose_msg.num_channels*sizeof(float));
        float tmp;
        for(int i = 0; i < pose_msg.num_channels; i++){
          tmp = my_set_points[i];
          pose_msg.channels[i] = tmp;
        }
      pose_t_publish((lcm_t *)lcm, "SET_POINT", &pose_msg);
      free(pose_msg.channels);
}


void publish_pid_state(pid_state_t* pid){
        pid_status_t pid_msg;
        pid_msg.utime =  ((double)utime_now())/1000000.0;

        pid_msg.kp = pid->kp;
        pid_msg.ki = pid->ki;
        pid_msg.kd = pid->kd;

        pid_msg.iterm = pid->iterm;
        pid_msg.prev_time = pid->prev_time;

        // Control Flags
        pid_msg.reached_target = pid->reached_target;
        pid_msg.first_time = pid->first_time;

        pid_msg.output = pid->output;
        pid_msg.error = pid->error;
        pid_msg.error_dot = pid->error_dot;
        pid_msg.x_ref = pid->x_ref;
        pid_msg.x_ref_dot = pid->x_ref_dot;

        // send lcm message to motors
       //puts("Publishing..");
        pid_status_t_publish((lcm_t *)lcm, pid->name, &pid_msg);
}

void spoof_transmitter(void){

  channels_t new_msg;
  new_msg.utime = 0.0;
  new_msg.num_channels = 8;
  new_msg.channels = (int16_t*) malloc(new_msg.num_channels*sizeof(int16_t));
  for(int i = 0; i < new_msg.num_channels; i++){
    new_msg.channels[i] = 0.0;
  }
  channels_t_publish((lcm_t *) lcm, "CHANNELS_1_RX", &new_msg);
  free(new_msg.channels);
}

bool check_alignment_manual(){
  pthread_mutex_lock(&state_mutex);
  bool safe = false;
  float manual_tol_y = 0.03;
  float manual_tol_z = 0.07;
  float dy = fabs(state->pose[1] - perch_pose[1]);
  float dz = fabs(perch_pose[2] - state->pose[2]); // z decreases as you

  if(dy < manual_tol_y && dz < manual_tol_z) safe = true;
  pthread_mutex_unlock(&state_mutex);
  return safe;

}

// Check if current pose is safe
bool check_safe(){
  pthread_mutex_lock(&state_mutex);
  bool safe = false;
  float dx = fabs(state->pose[0] - perch_pose[0]);
  float dy = fabs(state->pose[1] - perch_pose[1]);
  float dr = sqrt(dx*dx+dy*dy);

  float dz = perch_pose[2] - state->pose[2]; // z decreases as you

  float r0 = 0.2;
  float alpha = 1.0;
  if(dr < r0+alpha*dz) safe = true;
  pthread_mutex_unlock(&state_mutex);
  return safe;
}

// Set pose as setpoint
void set_curr_pose(void)
{
  pthread_mutex_lock(&state_mutex);
  for(int i = 0; i < 8; i++){
      my_set_points[i] = (float) state->pose[i];
    }
  pthread_mutex_unlock(&state_mutex);
}

// Set current point as perch point
void  set_curr_pt_perch(void){

    PID_reset_all();
  for(int i = 0; i < 8; i++){
      perch_pose[i] = (float) state->pose[i];
      safe_pose[i] = (float) state->pose[i];
    }
  safe_pose[2] = perch_pose[2] - Z_SAFE;
}


// Set pose as setpoint
void set_safe_point(void)
{
  pthread_mutex_lock(&state_mutex);
  for(int i = 0; i < 8; i++){
      my_set_points[i] = (float) safe_pose[i]; // XXX:TODO - set safe_point }
}
  pthread_mutex_unlock(&state_mutex);
}


// Set pose as setpoint
void set_perch_point(void)
{
  pthread_mutex_lock(&state_mutex);
  for(int i = 0; i < 8; i++){
      my_set_points[i] = (float) perch_pose[i]; // XXX:TODO - set safe_point
  }
  pthread_mutex_unlock(&state_mutex);
}

bool grab_set_point()
{

  double* wp = wp_queue_pop(waypoint_queue);
  if(wp){
      doubles_print(wp);
        for(int i=0;i<8;i++){
          my_set_points[i] = wp[i];
        }
            floats_print(my_set_points);
        publish_set_point();

    }
      PID_reset_all();
      return wp!=NULL;
}

void handle_key_input(char c){
  int hz = IN_FREQ;

  pthread_mutex_lock(&state_mutex);
  printf("The char you typed was %c\n", c);
 switch(c){
     case 'a':
     {
        //state->fence_on = 1; // Auto on
        // printf("auto on\n");
        // printf("%i\n", state->fence_on);
        auto_state = 1;
        break;
      }
      case 'k':{
        // state->fence_on = 0; // Auto off
        auto_state = 0;
        break;
      }
      case 'p':
       {
        print_state();
        break;
      }
      case 'c':
       {
        PID_print(pid_thrust);
        PID_print(pid_roll);
        PID_print(pid_pitch);
        PID_print(pid_yaw);
        break;
      }
       case 'r':
       {
        PID_reset_all();
        break;
      }
       case 's':
       {
            set_curr_pt_perch();
       }
       /*
      case 'w':
        {
        double tmp[8];
        for(int i=0;i<8;i++){
          tmp[i] = state->pose[i];
        }
        //tmp[2] +=1.0;
        //tmp[6] +=0.1;
        wp_queue_push(waypoint_queue,tmp);
        break;
      }
      case 's':
        {
          grab_set_point();
        break;
      }
      case 'q':
        {
        wp_queue_pop(waypoint_queue);
        break;
      }
      */
      // GRIPPER COMMANDS
      case '0':
        {
        commandGripper = 0;

        break;
      }
      case '1':
        {
        commandGripper = 1;
        break;
      }
      case '2':
        {
        commandGripper = 2;
        break;
      }
      case '3':
        {
        commandGripper = 3;
        break;
      }

      case '4':
        {
        commandGripper = 4;
        break;
      }

      case 'm':
        {
        //imu_grab_mocap = true;
        manual_perch_mode = true;
        break;
      }
     case 't':
        {
        //imu_grab_mocap = true;
        turn_off_propellers = false;
        take_off_flag = true;
        break;
      }

  }
   pthread_mutex_unlock(&state_mutex);
       usleep(1000000/hz);


}


void *key_in_loop(void* data){
  char inp;
  while(1){
    fseek(stdin,0,SEEK_END);
    scanf("%c", &inp);
    handle_key_input(inp);
    usleep(10000);
    //fflush(stdin);
    //printf("The char you typed was %c\n", inp);
  }
  return 0;
}

