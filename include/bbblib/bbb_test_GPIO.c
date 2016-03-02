 /*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *
  *  bbb_test_GPIO.c:  Test GPIO (toggle one output)
  *
  */

#define EXTERN  // Needed for global data declarations in bbb.h
#include "bbb.h"

int main()
{
  byte gpiopin=65; // GPIO_65 
  int i;

  // Initialize BBB and one GPIO pin

  if (bbb_init()) {
    printf("Error initializing BBB.\n");
    return -1;
  }

  if (bbb_initGPIO(gpiopin,gpio_output)) {
    printf("Error initializing BBB GPIO pin %d.\n", (int) gpiopin);
    return -1;
  }

  // Toggle GPIO pin 10 times (2 sec delay)

  for (i=0;i<10;i++) {

    bbb_toggleGPIO(gpiopin);
    printf("i=%d, gpio pin = %d\n", i, (int) bbb_readGPIO(gpiopin));

    sleep(2);
  }

  // Deinitialize GPIO
  bbb_deinitGPIO(gpiopin);

  return 0;
}
