 /*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://freecode.com/projects/blacklib
  */

#ifndef BBB_H_
#define BBB_H_

#include <ctype.h>
#include <errno.h>
#include <fcntl.h>   // For I2C
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>  // atoi() 
#include <dirent.h>  // dirent 
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>

// #include <linux/i2c.h>  // Has redefinitions not compatible with i2c-dev
#include <linux/i2c-dev.h>

//Type definitions
typedef struct {
      struct termios u;
}UART;

typedef unsigned char byte;

// ADC input (Available:  channels 0-6; 7 reserved).
#define AIN0 0
#define AIN1 1
#define AIN2 2
#define AIN3 3
#define AIN4 4
#define AIN5 5
#define AIN6 6

// GPIO type and value definitions
#define gpio_input  0  // gpio_type
#define gpio_output 1
#define gpio_low 0  // gpio_value
#define gpio_high 1
#define gpio_invalid 2  // gpio_type & gpio_value

// PWM channels by pin name
#define P9_14 0
#define P9_16 1
#define P8_19 2
#define P8_13 3
#define P9_42 4

// Added channel mapping for ROB 550 interface board
#define PWM_SV1 P9_14
#define PWM_SV2 P9_16
#define PWM_SV3 P8_19
#define PWM_SV4 P8_13
#define PWM_SV5 P9_42

// PWM stop/run & polarity
#define pwm_stop 0
#define pwm_run 1
#define pwm_straight 0
#define pwm_reverse 1
#define pwm_invalid 2

// I2C 
// Note:  Address specified by hardware device; BBB requires a 7-bit value so right shift by 1    
#define I2C_0 0  // I2C Name
#define I2C_1 1

#define I2C_READ 0x01
#define I2C_WRITE 0x00

#include "dynamixel.h"

////////////////////////////////////
// Data structure definition

struct I2C_data
{
  byte            name;            // I2C port name
  byte            address;         // I2C device address (7-bit value)
  byte            flags;           // O_RDONLY, O_WRONLY, O_RDWR, O_APPEND, O_TRUNC, O_NONBLOCK
  int             fd;              // I2C file descriptor 
  char            PortPath[50];    // I2C port path 
  byte            isOpenFlag;      // I2C file state (1=open, 0=closed)
};

///////////////////////////////////
// Function declarations

// BBB Initialization
int bbb_init();  // General initialization of BeagleBone Black (run first)
int searchDirectory(char *devdir, char *searchstr, char *returnstr);

// ADC Handling
int bbb_initADC();  // Initialize BeagleBone Black ADC
int bbb_readADC(byte ch);  // Read A/D channel (ch is AIN0 to AIN6)

// GPIO Handling
int bbb_initGPIO(byte pin, byte type);  // Init. BBB GPIO channel (type = gpio_input, gpio_output)
int bbb_deinitGPIO(byte pin);  // Deinit. BBB GPIO channel
int bbb_checkGPIOpin(byte pin);  // Returns 0 if pin is valid for GPIO, -1 otherwise
byte bbb_readGPIO(byte pin);  // Read GPIO input; returns 0 on success
int bbb_writeGPIO(byte pin, byte val); // Write GPIO output; returns 0 on success

// PWM Handling
int bbb_initPWM(byte pin);  // Initialization - call once for each PWM channel used
int bbb_setPeriodPWM(byte, unsigned int period); // Sets PWM period (in microseconds)
int bbb_setDutyPWM(byte, double percent); // Sets width of PWM signal
int bbb_setRunStatePWM(byte, byte); // Turns PWM on and off
int bbb_setPolarityPWM(byte, byte); // Sets PWM polarity
unsigned int bbb_getPeriodPWM(byte);  // Returns PWM period (in microseconds)
double bbb_getDutyPWM(byte);  // Returns percentage width of PWM
byte bbb_getRunStatePWM(byte);  // Gets PWM run state
byte bbb_getPolarityPWM(byte);  // Gets PWM polarity

// I2C Handling
int bbb_initI2C(struct I2C_data *i2cdata); // name, address, and flags should be preset
void bbb_deinitI2C(struct I2C_data *i2cdata);
int bbb_writeI2C(struct I2C_data *i2cdata, byte *writeBuffer, size_t bufsize);
int bbb_readI2C(struct I2C_data *i2cdata, byte *readBuffer, size_t bufsize);

// Global data
// #define EXTERN in your main() .c file
// #define EXTERN extern in all other .c files
EXTERN char capeMgrName[50], ocpName[50], ocpDir[50], slotsFilePath[100];
EXTERN char ADCDirName[50], ADCDir[100];
EXTERN byte gpio_stat[126];  // Status of GPIO initializations
EXTERN char pwmDir[7][50];  // PWM definitions (7 channels total)

// UART Handling
int initUART();
void closeUART(int fd);
int configUART(UART u, int property, char* value);
int txUART(int uart, unsigned char data, int n);
unsigned char rxUART(int uart);
int strUART(int uart, char* buf);


int set_gpio(int flag);
#endif
