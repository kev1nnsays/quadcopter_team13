/*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *
  *  bbb_gpio.c:  GPIO interface functions
  *   bbb_initGPIO(byte pin, byte type):  Initialization - call once for each GPIO pin used
  *   bbb_deinitGPIO(byte pin):  Deinitialization - call once for each GPIO pin used
  *   bbb_checkGPIOpin(byte pin):  Check that pin is a valid GPIO port number
  *   bbb_readGPIO(byte pin):  Read the value of GPIO pin
  *   bbb_writeGPIO(byte pin):  Write the value of GPIO pin
  *   bbb_toggleGPIO(byte pin):  Toggle the current GPIO pin value
  *
  */

#define EXTERN extern
#include "bbb.h"

int bbb_initGPIO(byte pin, byte type)
{
  FILE *fp_export, *fp_direction;
  char directionPath[100];

  // Verify pin is valid
  if (bbb_checkGPIOpin(pin)) {
    printf("bbb_initGPIO:  Pin %d is not a valid choice for GPIO.\n", (int) pin);
    return -1;
  }

  // Export GPIO pin
  if (!(fp_export = fopen("/sys/class/gpio/export", "w"))) {
    printf("bbb_initGPIO:  Error opening GPIO export stream.\n");
    return -1;
  }
  fprintf(fp_export,"%d", (int) pin);
  fclose(fp_export);

  // Set GPIO pin direction
  sprintf(directionPath,"/sys/class/gpio/gpio%d/direction", (int) pin);
  if (!(fp_direction = fopen(directionPath, "w"))) { 
    printf("bbb_initGPIO:  Error opening GPIO direction stream.\n");
    return -1;
  }
  if (type == gpio_input)  fprintf(fp_direction,"in");
  else                     fprintf(fp_direction,"out");
  fclose(fp_direction);

  gpio_stat[pin] = type;
  return 0;
}

int bbb_deinitGPIO(byte pin)  // Unexport GPIO pin
{
  FILE *fp_unexport;

  if (bbb_checkGPIOpin(pin)) {  // Check input GPIO pin #
    printf("bbb_deinitGPIO:  Illegal pin number of %d\n", (int) pin);
    return -1;
  } else if (gpio_stat[pin] == gpio_invalid) {
    printf("bbb_deinitGPIO:  Pin %d has invalid status.\n", (int) pin);
    return -1;
  }

  if (!(fp_unexport = fopen("/sys/class/gpio/unexport", "w"))) {
    printf("bbb_deinitGPIO:  Error opening GPIO unexport stream.\n");
    return -1;
  }
  fprintf(fp_unexport,"%d", (int) pin);
  fclose(fp_unexport);
  return 0;
}

int bbb_checkGPIOpin(byte pin)  // Returns 0 if pin is valid for GPIO; else returns -1
{
  if (pin > 89) {
    if (pin==117 || pin==120 || pin==121 || pin==122 || pin==123 || pin==125) return 0;  // OK
    else        return -1;  // Not OK
  } else if (pin > 59) {
    if (pin==82 || pin==83 || pin==84 || pin==85 || pin==64)      return -1;  // Not OK
    else      return 0; // OK
  } else if (pin > 49) {
    if (pin==51)      return 0;  // OK
    else              return -1; // Not OK
  } else if (pin > 29) {
    if (pin==41 || pin==42 || pin==43)      return -1;  // Not OK
    else      return 0;  // OK
  } else if (pin > 14) {
    if (pin==20 || pin==22 || pin==23 || pin==26 || pin==27)      return 0;  // OK
    else      return -1;  // Not OK
  } else {
    if (pin<2 || pin==6 || pin==12 || pin==13)       return -1;  // Not OK
    else      return 0;
  }
}
 
byte bbb_readGPIO(byte pin) 
{
  char gpio_str[100];
  FILE *fp_gpio;
  byte gpio_byte;

  if (bbb_checkGPIOpin(pin)) {  // Check GPIO pin #
    printf("bbb_readGPIO:  Illegal pin number of %d\n", (int) pin);
    return gpio_invalid;
  } else if (gpio_stat[pin] == gpio_invalid) {
    printf("bbb_readGPIO:  Pin %d does not have read (or write) status.\n", (int) pin);
    return gpio_invalid;
  }  

  sprintf(gpio_str,"/sys/class/gpio/gpio%d/value", (int) pin);
  if (!(fp_gpio = fopen(gpio_str, "r"))) {
    printf("bbb_readGPIO:  Error opening GPIO pin %d data stream.\n", (int) pin);
    return gpio_invalid;
  }
  fscanf(fp_gpio,"%s", gpio_str); // Maybe change this to a single fscanf (EMA)
  fclose(fp_gpio);
  gpio_byte = (byte) atoi(gpio_str);

  return gpio_byte;
}

int bbb_writeGPIO(byte pin, byte val) 
{
  char gpio_str[80];
  FILE *fp_gpio;

  if (bbb_checkGPIOpin(pin)) {  // Check GPIO pin #
    printf("bbb_writeGPIO:  Illegal pin number of %d\n", (int) pin);
    return -1;
  } else if (gpio_stat[pin] != gpio_output) {
    printf("bbb_writeGPIO:  Pin %d does not have write (output) status.\n", (int) pin);
    return -1;
  } 

  sprintf(gpio_str,"/sys/class/gpio/gpio%d/value", (int) pin);
  if (!(fp_gpio = fopen(gpio_str, "w"))) {
    printf("bbb_writeGPIO:  Error opening GPIO pin %d data stream.\n", (int) pin);
    return -1;
  }
  fprintf(fp_gpio,"%d", (int) val); // Maybe change this to a single fscanf of adcint (EMA)
  fclose(fp_gpio);

  return 0;
}
    
int bbb_toggleGPIO(byte pin)
{
  byte g = bbb_readGPIO(pin);

  if      (g == gpio_low)    bbb_writeGPIO(pin, gpio_high);
  else if (g == gpio_high)   bbb_writeGPIO(pin, gpio_low);
  else {
    printf("bbb_toggleGPIO:  Error reading pin %d.\n", (int) pin);
    return -1;
  } 
  return 0;
}
