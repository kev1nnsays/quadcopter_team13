// Processing loop (contains geofence code as an example only)
// processing_loop():  Top-level thread function
// update_set_points():  Geofencing reference points (may be useful as a guide)
// processing_loop_initialize():  Set up data processing / geofence variables
// read_config:  Read configuration parameters from a file (customize please)
//


// TODO - Add error in fsm
#define EXTERN extern
#define PROC_FREQ 100 // in Hz
#define GET_IMU 0
#define SAFE_HEIGHT 0.5
#define STABILITY_THRESH 3.0 //seconds
#define POSE_THRESHOLD 0.1 // m

#define SPEED_X 0.1 //m per s
#define SPEED_Y 0.1 //m per s
#define SPEED_Z 0.1 //m per s

#include "../include/quadcopter_main.h"
void read_config(char *);
int processing_loop_initialize();
int update_set_points(double* , float*);

/*
 * State estimation and guidance thread
*/
void *processing_loop(void *data){

  int hz = PROC_FREQ;
  processing_loop_initialize();

  // Local copies
  struct motion_capture_obs *mcobs[2];  // [0] = new, [1] = old
  struct state localstate;

  float watchdog_threshold = 0.2; // 10 samples
  while (1) {  // Main thread loop

    usleep(10);
    pthread_mutex_lock(&mcap_mutex);
    // Add watchdog
    mcobs[0] = mcap_obs;
    if(mcap_obs[1].time > 0) mcobs[1] = mcap_obs+1;
    else               mcobs[1] = mcap_obs;
    pthread_mutex_unlock(&mcap_mutex);

    // IMU input
    if(GET_IMU){
      localstate.time = (double)imudata.utime/1000000.0;
      for(int i=0;i<8;i++){
          localstate.pose[i] = imu_state->pose[i];
      }
    }
    else{

    //motion capture input
    localstate.time = mcobs[0]->time;
    localstate.pose[0] = mcobs[0]->pose[0]; // x position
    localstate.pose[1] = mcobs[0]->pose[1]; // y position
    localstate.pose[2] = mcobs[0]->pose[2]; // z position / altitude
    localstate.pose[3] = mcobs[0]->pose[5]; // yaw angle
    localstate.pose[4] = 0;
    localstate.pose[5] = 0;
    localstate.pose[6] = 0;
    localstate.pose[7] = 0;


    // Estimate velocities from first-order differentiation
    double mc_time_step = mcobs[0]->time - mcobs[1]->time;

    if(mc_time_step > watchdog_threshold){
      watchdog_panic = true;
      mocap_ready = false;
      puts("Panic");
    }
    else{
      watchdog_panic = false;
      mocap_ready = true;
    }

     //motion_capture input
    if(mc_time_step > 1.0E-7 && mc_time_step < 1){
      localstate.pose[4] = (mcobs[0]->pose[0]-mcobs[1]->pose[0])/mc_time_step;
      localstate.pose[5] = (mcobs[0]->pose[1]-mcobs[1]->pose[1])/mc_time_step;
      localstate.pose[6] = (mcobs[0]->pose[2]-mcobs[1]->pose[2])/mc_time_step;
      localstate.pose[7] = (mcobs[0]->pose[5]-mcobs[1]->pose[5])/mc_time_step;
    }

    //puts("POSE");
    //doubles_print(localstate.pose);
    //puts("----------");
    }
    // If auto mode, update state
    fsm_state_t fsm_msg;
    fsm_msg.utime =  ((double)utime_now())/1000000.0;
    double xxx =((double)utime_now())/1000000.0 - setpt_reached_time;
    fsm_msg.time_since_reached = xxx;
    fsm_msg.state_name = (char*)calloc(128,sizeof(char));
    fsm_msg.auto_state = auto_state;

    int fsm_state_numerical = -1; // MANUAL
    //if(fsm_quad_state==START_INIT) puts("\nStart Init State");
    strcpy(fsm_msg.state_name,"MANUAL MODE");
    for(int i=0;i<8;i++){
        fsm_msg.pose[i] = localstate.pose[i];
        fsm_msg.setpoint[i] = my_set_points[i];
    }
    fsm_msg.stable = setpt_stable==1;
    fsm_msg.reached = false;

    if(manual_perch_mode || auto_state == 1){
        int setpt_reached = update_set_points(localstate.pose, my_set_points);
        fsm_msg.reached = setpt_reached;
        fsm_msg.stable = setpt_stable;
        double time_now;
        switch(fsm_quad_state){
            case START_INIT:
                set_curr_pose(); // Sets waypoint to current pose
                fsm_quad_state = START_INIT_VERIFY;
                strcpy(fsm_msg.state_name,"START_INIT");
                fsm_state_numerical = 0;
                break;
            case START_INIT_VERIFY:
                if(manual_perch_mode){
                    fsm_quad_state = SAFE;
                }
                else{
                    if(setpt_stable) fsm_quad_state = START_GOTO_SAFE;
                    else fsm_quad_state = START_INIT_VERIFY;
                }
                strcpy(fsm_msg.state_name,"START_INIT_VERIFY");
                fsm_state_numerical = 1;
                break;
            case START_GOTO_SAFE:
                set_safe_point(); // Sets waypoint to sacred point above rod
                fsm_quad_state = START_SAFE_VERIFY;
                strcpy(fsm_msg.state_name , "START_GOTO_SAFE");
                fsm_state_numerical = 2;
                break;
            case START_SAFE_VERIFY:
                if(setpt_stable) fsm_quad_state = SAFE;
                else fsm_quad_state = START_GOTO_SAFE;
                strcpy(fsm_msg.state_name , "START_SAFE_VERIFY");
                fsm_state_numerical = 3;
                break;
            case SAFE:
                set_perch_point();
                fsm_quad_state = SAFE_PERCH_VERIFY;
                strcpy(fsm_msg.state_name , "SAFE");
                fsm_state_numerical = 4;
                break;
            case SAFE_PERCH_VERIFY:
                if(manual_perch_mode){
                    if(check_alignment_manual())
                        fsm_quad_state = PERCH;
                    else
                        fsm_quad_state = SAFE_PERCH_VERIFY;
                }
                else{
                    if(check_safe()){
                        if(setpt_stable) fsm_quad_state = PERCH;
                        else fsm_quad_state = SAFE_PERCH_VERIFY;
                    }
                    else{
                        fsm_quad_state = START_INIT;
                    }
                }
                fsm_state_numerical = 5;
                strcpy(fsm_msg.state_name , "SAFE_PERCH_VERIFY");
                break;
            case PERCH:
                commandGripper = 2;
                sleep(1);
                commandGripper = 3;
                sleep(1);
                fsm_quad_state = PERCH_VERIFY;
                strcpy(fsm_msg.state_name , "PERCH");
                fsm_state_numerical = 6;
                break;
            case PERCH_VERIFY:
                commandGripper = 4;
                sleep(1);
                commandGripper = 4;
                switch(dynam_perch_signal){
                    case 1:
                        fsm_quad_state = PERCHED_SUCCESS;
                        turn_off_propellers = true;
                        break;
                    case 0:
                        fsm_quad_state = PERCH_VERIFY;
                        break;
                    case -1:
                        commandGripper = 0;
                        sleep(2);
                        if(manual_perch_mode){
                            fsm_quad_state = SAFE;
                            puts("Goint to SAFE mode from Perch");
                        }
                        else
                            fsm_quad_state = START_INIT;
                        break;
                }
                strcpy(fsm_msg.state_name , "PERCH_VERIFY");
                fsm_state_numerical = 7;
                break;
            case PERCHED_SUCCESS:
                time_now = (double)(utime_now())/1000000.0;
                if(time_now - time_turned_off_cmd > 0.5){
                    // Motors turned off
                    turn_off_propellers = false;
                    fsm_quad_state = TAKE_OFF_INIT;
                }
                else{
                    fsm_quad_state = PERCHED_SUCCESS;
                }
                strcpy(fsm_msg.state_name , "PERCH_SUCCESS");
                fsm_state_numerical = 8;
                break;
            case TAKE_OFF_INIT:
                // Motors are turned off
                // Check for stable thrust
                if (take_off_flag){
                    turn_off_propellers = true;
                    fsm_quad_state = TAKE_OFF;
                }
                else fsm_quad_state = TAKE_OFF_INIT;
                strcpy(fsm_msg.state_name , "TAKE OFF_INIT");
                fsm_state_numerical = 9;
                break;
            case TAKE_OFF:
                // Open claws
                time_now = (double)(utime_now())/1000000.0;
                if(time_now- time_turned_off_cmd > 0.5){
                    set_safe_point();
                    turn_off_propellers = false;
                    fsm_quad_state = DONE;
                    sleep(2.5);// not sure.
                    commandGripper = 0;
                }
                else fsm_quad_state = TAKE_OFF;
                strcpy(fsm_msg.state_name , "TAKE OFF");
                fsm_state_numerical = 10;
                break;
            case DONE:
                fsm_quad_state = DONE;
                set_safe_point();
                strcpy(fsm_msg.state_name , "DONE");
                fsm_state_numerical = 11;
                break;
        }

    }

    fsm_state_t_publish((lcm_t *)lcm, "FSM", &fsm_msg);

    free(fsm_msg.state_name);
    fprintf(fsm_txt,"%ld,%lf,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
                fsm_msg.utime,
                fsm_msg.time_since_reached,
                fsm_msg.auto_state,
                fsm_state_numerical,
                fsm_msg.setpoint[0],
                fsm_msg.setpoint[1],
                fsm_msg.setpoint[2],
                fsm_msg.setpoint[3],
                fsm_msg.setpoint[4],
                fsm_msg.setpoint[5],
                fsm_msg.setpoint[6],
                fsm_msg.setpoint[7],
                fsm_msg.pose[0],
                fsm_msg.pose[1],
                fsm_msg.pose[2],
                fsm_msg.pose[3],
                fsm_msg.pose[4],
                fsm_msg.pose[5],
                fsm_msg.pose[6],
                fsm_msg.pose[7],
                fsm_msg.stable,
                fsm_msg.reached);
    fflush(fsm_txt);





    // Copy to global state (minimize total time state is locked by the mutex)
    pthread_mutex_lock(&state_mutex);
    memcpy(state, &localstate, sizeof(struct state));
    pthread_mutex_unlock(&state_mutex);

    publish_state();
    publish_set_point();
    //spoof_transmitter();
    publish_pid_state(pid_roll);
    publish_pid_state(pid_yaw);
    publish_pid_state( pid_pitch);
    publish_pid_state( pid_thrust);

    usleep(1000000/hz);

  } // end while processing_loop()

  return 0;
}

bool reached(double* pose, float *set_points){
  float max_abs_err = -999.;
  float curr,goal,err;
  for(int i=0;i<4;i++){
      goal = set_points[i];
      curr = pose[i];
      err = fabs(goal-curr);
      //printf("Error = %0.5f\n",err);

      max_abs_err = err>max_abs_err?err:max_abs_err;
  }
 printf("Max Error: %f out of %f\n",max_abs_err,POSE_THRESHOLD);

  return max_abs_err < POSE_THRESHOLD;
}

/**
 * update_set_points() :
 *      Reached: returns 1
 *      setpt_stable = 1 if stable.
 */
int update_set_points(double* pose, float *set_points){
    //TODO: Handle watchdog...
    if(reached(pose,set_points)){
        printf("Setpoint reached time = %lf\n",setpt_reached_time);

        double time_now;
        if(setpt_reached_time>0){
            time_now =  ((double)utime_now())/1000000.0;
            double time_stable = time_now - setpt_reached_time;
            if(time_stable>STABILITY_THRESH){
                printf("Time Stable = %f out of %f\n",time_stable,STABILITY_THRESH);
                setpt_stable = 1;
                PID_reset_all();
            }

        }
        else{
            time_now =  ((double)utime_now())/1000000.0;
            setpt_reached_time = time_now;
        }
        return 1;
    }
    else{
        // Individual pids..
        // Update setpoints to be target - pose
        if(1){
            float tmp =  fabs(set_points[0] - pose[0]);
            set_points[4] = SPEED_X*(set_points[0] - pose[0])/tmp;


            tmp =  fabs(set_points[1] - pose[1]);
            set_points[5] = SPEED_Y*(set_points[1] - pose[1])/tmp;

            tmp =  fabs(set_points[2] - pose[2]);
            set_points[6] = SPEED_Z*(set_points[2] - pose[2])/tmp;

        }
    }

    setpt_reached_time = -1.0;
    setpt_stable = 0;
    return 0;
}


/**
 * processing_loop_initialize()
*/
int processing_loop_initialize()
{
  // Initialize state struct
  state = (state_t*) calloc(1,sizeof(*state));
  state->time = ((double)utime_now())/1000000;
  memset(state->pose,0,sizeof(state->pose));


  quad_open_claw = false;
  take_off_flag = false;
  dynam_claw_open = false;
  dynam_perch_signal = 0;


    time_turned_off_cmd = -999.9f;

  manual_perch_mode = false;
  turn_off_propellers = false;
  take_off_flag = false;

  pid_thrust = (pid_state_t*) calloc(1,sizeof(pid_state_t));
  strcpy(pid_thrust->name,"Thrust");
  pid_thrust->track_id = 2;

  pid_yaw = (pid_state_t*) calloc(1,sizeof(pid_state_t));
  strcpy(pid_yaw->name,"Yaw");
  pid_yaw->track_id = 3;

  pid_pitch = (pid_state_t*) calloc(1,sizeof(pid_state_t));
  strcpy(pid_pitch->name,"Pitch");
  pid_pitch->track_id = 0;

  pid_roll = (pid_state_t*) calloc(1,sizeof(pid_state_t));
  strcpy(pid_roll->name,"Roll");
  pid_roll->track_id = 1;

  fsm_quad_state = START_INIT;

  pthread_mutex_lock(&waypoints_mutex);
  wp_queue_print(waypoint_queue);
  pthread_mutex_unlock(&waypoints_mutex);

  // Read configuration file
  char blah[] = "config.txt";
  read_config(blah);

  mcap_obs[0].time = state->time;
  mcap_obs[1].time = -1.0;  // Signal that this isn't set yet

  setpt_reached_time = -1.0;
  setpt_stable = 0;

  // Initialize Altitude Velocity Data Structures
  memset(diff_z, 0, sizeof(diff_z));
  memset(diff_z_med, 0, sizeof(diff_z_med));

  // Fence variables
  state->fence_on = 0;
  auto_state = 0;
  //printf("processing_loop set fence_on off");
  //memset(my_set_points,0,sizeof(my_set_points));
  state->time_fence_init = 0;

  // Initialize IMU data
  //if(imu_mode == 'u' || imu_mode == 'r'){
  //  imu_initialize();
  //}




  return 0;
}


void read_config_pid(FILE* conf,pid_state_t* pid_state)
{
  char str[1000];
  char* record;

  fgets(str,sizeof(str),conf);
  record = strtok(str,",");
  record = strtok(NULL,",");
  sscanf(record,"%f",&pid_state->kp);
  record = strtok(NULL,",");
  sscanf(record,"%f",&pid_state->ki);
  record = strtok(NULL,",");
  sscanf(record,"%f",&pid_state->kd);
  fgets(str,sizeof(str),conf);
  record = strtok(str,",");
  record = strtok(NULL,",");
  sscanf(record,"%f",&pid_state->trim);
  fgets(str,sizeof(str),conf);
  record = strtok(str,",");
  record = strtok(NULL,",");
  sscanf(record,"%f",&pid_state->isat);
  fgets(str,sizeof(str),conf);
  record = strtok(str,",");
  record = strtok(NULL,",");
  sscanf(record,"%f",&pid_state->outputsat);
  printf("PID  Gains = (%0.5f,%0.5f,%0.5f)\n",pid_state->kp,pid_state->ki,pid_state->kd);
  printf(" Trim = %0.5f\t ISat = %0.5f \t OutSat = %0.5f\n",pid_state->trim,pid_state->isat,pid_state->outputsat);

}

void read_config(char* config){
  // open configuration file
  FILE* conf = fopen(config,"r");
  // holder string
  char str[1000];
  char* record;


  // Start pos
  fgets(str,sizeof(str),conf);
  record = strtok(str,",");
  record = strtok(NULL,",");
  sscanf(record,"%lf",&start[0]);
  record = strtok(NULL,",");
  sscanf(record,"%lf",&start[1]);
  printf("Quadcopter Start = (%0.5f,%0.5f)\n",start[0],start[1]);

// Target Positions
  fgets(str,sizeof(str),conf);
  fgets(str,sizeof(str),conf);
  record = strtok(str,",");
  record = strtok(NULL,",");
  sscanf(record,"%f",&target_pt1[0]);
  record = strtok(NULL,",");
  sscanf(record,"%f",&target_pt1[1]);
  record = strtok(NULL,",");
  sscanf(record,"%f",&target_pt1[2]);
  printf("Target Pt1 = (%0.5f,%0.5f,,%0.5f)\n",target_pt1[0],target_pt1[1],target_pt1[2]);

  // Target Positions
  fgets(str,sizeof(str),conf);
  record = strtok(str,",");
  record = strtok(NULL,",");
  sscanf(record,"%f",&target_pt2[0]);
  record = strtok(NULL,",");
  sscanf(record,"%f",&target_pt2[1]);
  record = strtok(NULL,",");
  sscanf(record,"%f",&target_pt2[2]);
  printf("Target Pt2 = (%0.5f,%0.5f,,%0.5f)\n",target_pt2[0],target_pt2[1],target_pt2[2]);

  target_z = target_pt1[2]+target_pt1[2];
  target_x = target_pt1[0]+target_pt1[0];
  target_y = target_pt1[1]+target_pt1[1];

  target_z/=2.0;
  target_x/=2.0;
  target_y/=2.0;

  printf("Target Z = %0.5f\n",target_z);

  // Thrust PID
  fgets(str,sizeof(str),conf);
  read_config_pid(conf,pid_thrust);
  fgets(str,sizeof(str),conf);
  read_config_pid(conf,pid_roll);
  fgets(str,sizeof(str),conf);
  read_config_pid(conf,pid_pitch);
  fgets(str,sizeof(str),conf);
  read_config_pid(conf,pid_yaw);
  fclose(conf);
}
