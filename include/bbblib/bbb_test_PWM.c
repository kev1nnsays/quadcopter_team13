 /*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *
  *  bbb_test_PWM.c:  Tests PWM for one servo channel
  *
  */

#define EXTERN  // Needed for global data declarations in bbb.h
#include "bbb.h"

//#define PWM_CHOICE PWM_SV2  // set up choice of channel for each test

int main()
{
  int i, perc;
  int PWM_CHOICE;
  printf("Enter PWM channel #.\n");
  scanf("%d", &PWM_CHOICE);

  // Initialize BBB and one PWM channel

  if (bbb_init()) {
    printf("Error initializing BBB.\n");
    return -1;
  }

  if (bbb_initPWM(PWM_CHOICE)) {
    printf("Error initializing BBB PWM pin %d. Are you running as root?\n", PWM_CHOICE);
    return -1;
  }

  // Setup PWM (ask user for duty cycle %)
  printf("Enter PWM duty cycle in percent (0-100):\n");
  scanf("%d", &perc);

  // The PWM hardware is set up so that channels 0 and 1 share the same period
  // and run state, and channels 2 and 3 are similarly paired
  bbb_setPeriodPWM(PWM_CHOICE, 20000000);  // Period is in nanoseconds
  bbb_setDutyPWM(PWM_CHOICE, (double) perc);  // Duty cycle percent (0-100) (UPDATED 11/12)
  bbb_setRunStatePWM(PWM_CHOICE, pwm_run);

  printf("Running PWM at %.1f percent for 10 seconds...\n",
	 bbb_getDutyPWM(PWM_CHOICE));

  sleep(10);  // Run for 10 seconds

  // Stop PWM & exit

  bbb_setDutyPWM(PWM_CHOICE, 0);

  return 0;
}
