/*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  Consult i2c-tools page (http://www.lm-sensors.org/wiki/I2CTools).  
  *   -->Useful command: i2cdetect -r 2  (for I2C2)
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *      http://datko.net/2013/11/03/bbb_i2c/
  *
  *  bbb_i2c.c:  I2C Interface functions
  *  -----------------------------------
  *   bbb_initI2C():  Initialize I2C
  *   bbb_deinitI2C():  De-initialize I2C
  *   bbb_writeI2C():  Uses write() to output line of I2C data 
  *   bbb_readI2C():  Uses read() to input line of I2C data
  *
  */

#define EXTERN extern
#include "bbb.h"
  
// Preset is required for the following values:
// i2cdata->name (e.g., I2C_1)
// i2cdata->address (see your I2C device documentation) 
// i2cdata->flags (e.g., O_RDWR)
//
int bbb_initI2C(struct I2C_data *i2cdata) 
{
  sprintf(i2cdata->PortPath, "/dev/i2c-%d", (int) (i2cdata->name));
  if ((i2cdata->fd = open(i2cdata->PortPath, (int) (i2cdata->flags))) < 0) {
    printf("Error opening I2C port %s\n", i2cdata->PortPath);
    i2cdata->isOpenFlag = 0;  // logical false
    return -1;
  } 
  i2cdata->isOpenFlag = 1;  // logical true
  return 0;  // Status (0=success)
}

void bbb_deinitI2C(struct I2C_data *i2cdata) 
{
  if (i2cdata->isOpenFlag)    close(i2cdata->fd);
}

int bbb_writeI2C(struct I2C_data *i2cdata, byte *writeBuffer, size_t bufsize)
{
  if (ioctl(i2cdata->fd, I2C_SLAVE, (i2cdata->address >> 1))) {
    printf("Error in initI2C (ioctl)\n");
    return -1;
  }

  if (write(i2cdata->fd, writeBuffer, bufsize) != bufsize) { // Write data
    printf("Error writing data to I2C.\n");
    return -1;
  } 
  return 0;
}

int bbb_readI2C(struct I2C_data *i2cdata, byte *readBuffer, size_t bufsize)
{
  if (ioctl(i2cdata->fd, I2C_SLAVE, (i2cdata->address >> 1))) {
    printf("Error in initI2C (ioctl)\n");
    return -1;
  }

  if (read(i2cdata->fd, readBuffer, bufsize) < 0) { // Read data
    printf("Error reading line from I2C.\n");
    return -1;
  }
  return 0;
}
