// Controller_PWM_Read.c

// This code reads in the PWM data (PWM periods, ms) produced by the controller. The LCM instance for this data is
// channels_t. Here is information concerning the channels data contained in a channels_t struct:

// Channel 0: Pitch, 1074 (Pulled Back) – 1489 (Center) – 1918 (Pushed Forward)
// Channel 1: Yaw, 1920 (Left) – 1498 (Center) – 1075 (Right)
// Channel 2: Roll, 1915 (Left) – 1508 (Center) – 1075 (Right)
// Channel 3: Thrust, 1075 (Pulled Back, No Thrust) – 1498 (Center) – 1918 (Pushed Forward, Full Thrust)
// Channel 4: None.
// Channel 5: None.
// Channel 6: Unknown.
// Channel 7: Autopilot Switch, 1075 (Pushed Forward, O Gear, Manual), 1918 (Pulled Back, 1 F Mode, Autopilot)

//-------------------------------------------------------
//Including Relevant Files
//-------------------------------------------------------

//General Header Files
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <lcm/lcm.h>
#include <pthread.h>

//LCM Header File
#include "../lcmtypes/channels_t.h"

//Other Local Header Files
#include "../include/filter_util.h"
#include "../include/util.h"

//-------------------------------------------------------
//Defining Function Global Variables
//-------------------------------------------------------

#define EXTERN

// Data buffer structure: 
/*Note: I am note entirely sure that this format will work, since it
  requires each handler to pass the message's address to the buffer to
  later be accessed by the processing thread. Since these messages are
  passed by reference I think that this is valid to pass each
  messages' address to this stucture from the process, but I am not
  positive. We shall see. */

typedef struct
{
  channels_t *msg_conpwm;
} DAT_BUF;

DAT_BUF BUFFER;
bool print_to_screen = true;

//Timing Variables -------------------------------------------------------
int64_t conpwm_start_time; 

//Dummy processing variable ----------------------------------------------

//Mutexes ----------------------------------------------------------------
pthread_mutex_t conpwm_data_mutex;

//Text Files -------------------------------------------------------------
FILE* conpwm_file;

//-------------------------------------------------------
//Specifying LCM Handler Procedures
//-------------------------------------------------------

//Controller PWM
void conpwm_handler(const lcm_recv_buf_t *rbuf, const char *channel,
		 const channels_t *msg, void *userdata)
{
  // Compute the time of the measurement relative to start time
  double time_since;
  time_since = (double)(msg->utime - conpwm_start_time);
  time_since = time_since / 1000000;

  // Print Screen Operations --------------------------------
  //bool print_to_screen = false;
  if (print_to_screen == true)
     {
       printf("Received Message on Channel %s, Timestamp %" PRId64 "\n", channel, msg->utime);
        printf("Thrust: (%d), Pitch: (%d), Roll: (%d), Yaw: (%d), Mode: (%d)\n\n",
		msg->channels[3], msg->channels[0], msg->channels[2], msg->channels[1], msg->channels[7]);
     }

  // Text File Operations ------------------------------------
  fprintf(conpwm_file,"%.3f,%d,%d,%d,%d,%d\n",time_since,msg->channels[3],msg->channels[0],msg->channels[2],msg->channels[1],msg->channels[7]);
  fflush(conpwm_file);

  /*
  // Buffer Operations ---------------------------------------

  pthread_mutex_lock(&conpwm_data_mutex);

  // Write the most recent Controller PMW message to data buffer.
  //BUFFER.msg_conpwm = msg;

  //printf("CONPWM\n\n");

  pthread_mutex_unlock(&conpwm_data_mutex);
  */

}

void* conpwm_receive_loop(void *data)
{
 //Opening the text file
  conpwm_file = fopen("conpwm_data.txt","w"); // Mode "w" for overwriting a file.

  //Define LCM channels and subscribe to messages
  lcm_t* lcm_CONPWM = lcm_create(NULL);
  channels_t_subscribe(lcm_CONPWM, "CHANNELS_1_RX", conpwm_handler,NULL);

  //Recording the start time of the controller's PWM operation
    conpwm_start_time = utime_now();

  //Handling messages
  while(1)
    {
      lcm_handle(lcm_CONPWM);
      usleep(100000);
    }

  fclose(conpwm_file);
  lcm_destroy(lcm_CONPWM);
}

//Main Function
//--------------------------------------------------------
int main()
{
  // Initialize the mutex
  pthread_mutex_init(&conpwm_data_mutex,NULL);
  
  /* Currently the two-thread environment is suspended. In order to
     activate it, uncomment this block of code as well as
     "lcm_receive_thread" above.  Furthermore, comment out the
     thread environment below, as well as the associated threaded
     processes above.

  //Two-Thread Environment ---------------------------------------
  // Start the thread
  pthread_t lcm_receive_thread, processing_thread;
  pthread_create(&lcm_receive_thread,NULL,lcm_receive_loop,NULL);
  pthread_create(&processing_thread,NULL,processing_loop,NULL);

  //Join both threads and let them do their thing
  pthread_join(lcm_receive_thread,NULL);
  pthread_join(processing_thread,NULL);

  */

  //Four-Thread Environment --------------------------------------
  // Create the threads
  pthread_t conpwm_receive_thread;
  pthread_create(&conpwm_receive_thread,NULL,conpwm_receive_loop,NULL);

  // Join the threads
  pthread_join(conpwm_receive_thread,NULL);
  
  return 0;  
}
