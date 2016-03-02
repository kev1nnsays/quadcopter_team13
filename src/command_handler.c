// Handler function to either pass through RX commands to Naza or else
// copy computer (autonomous control) commands through to Naza.
#define EXTERN extern

#include "../include/quadcopter_main.h"
#define AUTO 1500

////////////////////////////////////////////////////////////////////////

void channels_handler(const lcm_recv_buf_t *rbuf, const char *channel,
		      const channels_t *msg, void *userdata)
{
  // create a copy of the received message
  channels_t new_msg;
  new_msg.utime = msg->utime;
  new_msg.num_channels = msg->num_channels;
  new_msg.channels = (int16_t*) malloc(msg->num_channels*sizeof(int16_t));
  for(int i = 0; i < msg->num_channels; i++){
    new_msg.channels[i] = msg->channels[i];
  }

  // Copy state to local state struct to minimize mutex lock time
  struct state localstate;
  pthread_mutex_lock(&state_mutex);
  memcpy(&localstate, state, sizeof(struct state));
  pthread_mutex_unlock(&state_mutex);

  int auto_cmd = new_msg.channels[7];
  // if(auto_cmd > AUTO) localstate.fence_on = 1;  // Set AUTO ON

  // else localstate.fence_on = 0;  // Set AUTO OFF
  if(1){
      if(auto_cmd > AUTO) auto_state = 1;  // Set AUTO ON
      else {
          auto_state = 0;  // Set AUTO OFF
          fsm_quad_state = START_INIT;
          PID_reset_all();

      }
  }

  // Decide whether or not to edit the motor message prior to sending it
  // set_points[] array is specific to geofencing.  You need to add code
  // to compute them for our FlightLab application!!!
  float pose[8], set_points[8];
  // if(localstate.fence_on == 1){
  if((auto_state == 1 || manual_perch_mode) && turn_off_propellers){
          new_msg.channels[0] = 1080;
          new_msg.channels[1] = 1920;
          new_msg.channels[2] = 1080;
          new_msg.channels[3] = 1080;
          new_msg.channels[7] = 1180;
          time_turned_off_cmd = (double)utime_now()/1000000.0;
  }
  else{
      if(auto_state == 1){
          //printf("auto_control start\n");
          for(int i = 0; i < 8; i++){
              pose[i] = (float) localstate.pose[i];
              set_points[i] = my_set_points[i];
          }
          // This needs to change - mutex held way too long
          auto_control(pose, set_points, new_msg.channels);

      } else{  // Fence off
          // pass user commands through without modifying
          new_msg.channels[7] = 1180;
      }
  }

new_msg.channels[7] = 1180;
  // send lcm message to motors
  channels_t_publish((lcm_t *) userdata, "CHANNELS_1_TX", &new_msg);

   int64_t utime =  ((double)utime_now())/1000000.0;
  // Save received (msg) and modified (new_msg) command data to file.
  // NOTE:  Customize as needed (set_points[] is for geofencing)
  fprintf(block_txt,"%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f\n",
	  (long int) msg->utime,msg->channels[0],msg->channels[1],msg->channels[2],
	  msg->channels[3], msg->channels[7],
	  new_msg.channels[0],new_msg.channels[1],new_msg.channels[2],
	  new_msg.channels[3],new_msg.channels[7],
	  set_points[0],set_points[1],set_points[2],
	  set_points[3],set_points[4],set_points[5],set_points[6],
	  set_points[7], pose[0],pose[1],pose[2],
    pose[3],pose[4],pose[5],pose[6],
    pose[7]);


  fprintf(pid_txt,"%f,%lf,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%ld\n",
        pid_pitch->iterm,
        pid_pitch->prev_time,
        pid_pitch->reached_target,
        pid_pitch->first_time,
        pid_pitch->error,
        pid_pitch->x_ref,
        pid_pitch->output,
        pid_pitch->kp,
        pid_pitch->kd,
        pid_pitch->ki,
        pid_pitch->error_dot,
        pid_pitch->x_ref_dot,
        utime
  );
  fprintf(pid_txt,"%f,%lf,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%ld\n",
        pid_roll->iterm,
        pid_roll->prev_time,
        pid_roll->reached_target,
        pid_roll->first_time,
        pid_roll->error,
        pid_roll->x_ref,
        pid_roll->output,
        pid_roll->kp,
        pid_roll->kd,
        pid_roll->ki,
        pid_roll->error_dot,
        pid_roll->x_ref_dot,
        utime
  );
  fprintf(pid_txt,"%f,%lf,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%ld\n",
        pid_yaw->iterm,
        pid_yaw->prev_time,
        pid_yaw->reached_target,
        pid_yaw->first_time,
        pid_yaw->error,
        pid_yaw->x_ref,
        pid_yaw->output,
        pid_yaw->kp,
        pid_yaw->kd,
        pid_yaw->ki,
        pid_yaw->error_dot,
        pid_yaw->x_ref_dot,
        utime
  );
  fprintf(pid_txt,"%f,%lf,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%ld\n",
        pid_thrust->iterm,
        pid_thrust->prev_time,
        pid_thrust->reached_target,
        pid_thrust->first_time,
        pid_thrust->error,
        pid_thrust->x_ref,
        pid_thrust->output,
        pid_thrust->kp,
        pid_thrust->kd,
        pid_thrust->ki,
        pid_thrust->error_dot,
        pid_thrust->x_ref_dot,
        utime
  );

  fflush(block_txt);
  fflush(pid_txt);
}
