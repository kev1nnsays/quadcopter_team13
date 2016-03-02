// Handler function to either pass through RX commands to Naza or else
// copy computer (autonomous control) commands through to Naza.
#define EXTERN extern

#include "../include/quadcopter_main.h"
#define AUTO 50000

////////////////////////////////////////////////////////////////////////

void wp_trigger_handler(const lcm_recv_buf_t *rbuf, const char *channel,
		      const waypoint_trigger_t *msg, void *userdata)
{

  puts("Trigger Received!!");

  // Copy state to local state struct to minimize mutex lock time
  struct state localstate;
  pthread_mutex_lock(&state_mutex);
  memcpy(&localstate, state, sizeof(struct state));
  pthread_mutex_unlock(&state_mutex);


}
