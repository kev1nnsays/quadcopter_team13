 /*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *
  *  bbb_test_ADC.c:  Tests ADC functionality
  *
  */

#define EXTERN  // Needed for global data declarations in bbb.h
#include "bbb.h"

int main()
{
  int adc_value, i;

  // Initialization

  if (bbb_init()) {
    printf("Error initializing BBB.\n");
    return -1;
  }

  if (bbb_initADC()) {
    printf("Error initializing BBB ADC.\n");
    return -1;
  }

  // Main loop:  read ADC and print result, 10 times (once per second)

  for (i=0;i<10;i++) {
    adc_value = bbb_readADC(AIN0);
    printf("i=%d, adc_value (millivolts) = %d\n", i, adc_value); 
    sleep(1);
  }

  // No deinitialization necessary for ADC only (you can include it if you'd like though)

  return 0;
}
