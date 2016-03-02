#ifndef ROB550_STRUCT
#define ROB550_STRUCT

// Shared data structures for ROB 550 Quadrotor project (not LCM)
typedef enum {START_INIT,START_INIT_VERIFY,START_GOTO_SAFE,START_SAFE_VERIFY,
              SAFE,SAFE_PERCH_VERIFY,PERCH,PERCH_VERIFY,PERCHED_SUCCESS,
              TAKE_OFF_INIT,TAKE_OFF,DONE} quad_state_t;

typedef struct imu imu_t;
struct imu  // Only includes gyro and accel data for now
{
  double utime;
  double gyro_x, gyro_y, gyro_z;
  double accel_x, accel_y, accel_z;
};

typedef struct logical_state logical_state_t;
struct logical_state
{
  /* data */
  int state;

};


typedef struct imu_state imu_state_t;
struct imu_state{
   bool ready;
   bool first_time;

   imu_t prev_reading;
   double pose[8];
};

/*
 * state:
 * struct holds values related to current state estimate and
 * autonomous control (fence) activity
*/
typedef struct state state_t;
struct state{
  double time;

  // aircraft position (x,y,alt,yaw,xdot,ydot,altdot,yawdot)
  double pose[8];


  // Flag indicating whether the autonomous controller is activ or not
  // (1 = on; 0 = pass through pilot commands)
  int fence_on;

  // Geofence specific variables
  float set_points[8];
  double time_fence_init;
  logical_state_t fsm_state;
};


typedef struct pid_state pid_state_t;
struct pid_state{
  // Tune Settings
  float kp;
  float ki;
  float kd;

  float trim;
  float isat;
  float outputsat;

  // Internal maintenance
  float iterm;
  double prev_time;
  float x_ref;
  float x_ref_dot;

  // Control Flags
  int reached_target;
  bool first_time;

  // Bookkeeping
  float error;
  float error_dot;
  float output;
  int track_id;

  // Name
  char name[128];
};
#endif
