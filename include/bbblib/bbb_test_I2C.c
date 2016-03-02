 /*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *      https://www.nordevx.com/content/lsm9ds0-accelerometer-magnetometer-and-gyro-one
  *
  *  bbb_test_I2C.c:  Code to read and print data from
  *  the Adafruit LSM9DS0 gyro/accelerometer/magnetometer IMU
  * 
  */ 

//////  WAIT TO DOWNLOAD - WILL UPDATE!!! ///////

#define EXTERN  // Needed for global data declarations in bbb.h
#include "bbb.h" 

// Local functions
void read_gyro(struct I2C_data *i2cdata, double g[]);
void read_accel(struct I2C_data *i2cdata, double a[]);
void read_mag(struct I2C_data *i2cdata, double m[]);
void read_temp(struct I2C_data *i2cdata, double *temp);

// Define gyro bias data
// ADJUST THESE FOR YOUR IMU (see read_gyro() code below)
#define GYROX_BIAS  130
#define GYROY_BIAS  -35
#define GYROZ_BIAS  750

int main()
{
  struct I2C_data i2cd_gyro, i2cd_accelmag;
  int i;
  byte buf[10];
  double gyro[3], accel[3], mag[3], temperature;

  // Initialize BBB & I2C ports

  if (bbb_init()) {
    printf("Error initializing BBB.\n");
    return -1;
  }

  i2cd_gyro.name = I2C_1;  // I2C2 is enumerated as 1 on the BBB unless I2C1 is enabled
  i2cd_gyro.address = 0xD6; // Gyro address (right shifted by 1 to get 7-bit value)
  i2cd_gyro.flags = O_RDWR;  

  i2cd_accelmag.name = I2C_1;  // Same I2C port as gyro
  i2cd_accelmag.address = 0x3A; // Accel/Mag address (right shifted by 1 to get 7-bit value)
  i2cd_accelmag.flags = O_RDWR;  
  usleep(1000);

  if (bbb_initI2C(&i2cd_gyro)) { // Open I2C fd for gyro
    printf("Error initializing I2C port %d for gyro.\n", (int) (i2cd_gyro.name));
    return -1;
  }
  usleep(1000);
  
  if (bbb_initI2C(&i2cd_accelmag)) { // Open a second fd (same I2C) for accelmag
    printf("Error initializing I2C port %d for accel.\n", (int) (i2cd_accelmag.name));
    return -1;
  }
  usleep(1000);

  // Activate accelerometer, magnetometer, and gyro

  buf[0] = 0x20; buf[1] = 0x0F; 
  bbb_writeI2C(&i2cd_gyro, buf, 2);
  usleep(1000);

  // XM control registers: 0x1F - 0x26
  buf[0] = 0x20; buf[1] = 0x67;  // CTRL_REG1_XM:  3=12.5Hz; 7=enable all accel ch.
  bbb_writeI2C(&i2cd_accelmag, buf, 2); 
  usleep(1000);
  buf[0] = 0x24; buf[1] = 0xF0;  // Activate accel, temp through control register 5
  bbb_writeI2C(&i2cd_accelmag, buf, 2); 
  usleep(1000);
  buf[0] = 0x26; buf[1] = 0x00;  // Send 0x00 to control register 7
  bbb_writeI2C(&i2cd_accelmag, buf, 2); 
  usleep(1000);
  
  // Read & print I2C data from 9DOF IMU 10 times (1 second delay)
  for (i=0;i<10;i++) {
    read_gyro(&i2cd_gyro, gyro);
    printf("gyro (deg/sec) = (%.3lf, %.3lf, %.3lf), ", gyro[0], gyro[1], gyro[2]);
    usleep(1000);
    
    read_accel(&i2cd_accelmag, accel);
    printf("accel (g) = (%.3lf, %.3lf, %.3lf), ", accel[0], accel[1], accel[2]);
    usleep(1000);
    
    /* Included for completeness
    read_mag(&i2cd_accelmag, mag);
    printf("mag (not useful) = (%.3lf, %.3lf, %.3lf), ", mag[0], mag[1], mag[2]);
    usleep(1000); */

    read_temp(&i2cd_accelmag, &temperature);
    printf("T (deg C) = %.3lf\n", temperature);

    sleep(1);
  }
  
  //bbb_deinitI2C(&i2cd_gyro);
  bbb_deinitI2C(&i2cd_accelmag);

  return 0;
}

//////////////////////////////////////////
void read_gyro(struct I2C_data *i2cd, double gyro[])
{
  short tempint;
  byte buf, lobyte, hibyte;  // Used to store I2C data
  
  buf = 0x28; // X gyro, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  usleep(1000);
  bbb_readI2C(i2cd, &lobyte, 1);
  usleep(1000);
  buf = 0x29; // X gyro, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  usleep(1000);
  bbb_readI2C(i2cd, &hibyte, 1);
  usleep(1000);
  tempint = (((short) hibyte) << 8) | lobyte;
  // GYROX_BIAS is the zero bias/offset - please adjust to make tempint close to zero for your x-gyro
  // printf("gyro x=%hd\n", tempint+GYROX_BIAS); // With your bias this should be near zero
  gyro[0] = 0.00875*(tempint+GYROX_BIAS);  // 110 is the zero bias/offset - please adjust
  
  buf = 0x2A; // Y gyro, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  usleep(1000);
  bbb_readI2C(i2cd, &lobyte, 1);
  usleep(1000);
  buf = 0x2B; // Y gyro, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  usleep(1000);
  bbb_readI2C(i2cd, &hibyte, 1);
  usleep(1000);
  tempint = (((short) hibyte) << 8) | lobyte;
  // GYROY_BIAS is the zero bias/offset - please adjust to make tempint close to zero for your y-gyro
  // printf("gyro y=%hd\n", tempint + GYROY_BIAS); // With your bias this should be near zero
  gyro[1] = 0.00875*(tempint + GYROY_BIAS);  
  
  buf = 0x2C; // Z gyro, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  usleep(1000);
  bbb_readI2C(i2cd, &lobyte, 1);
  usleep(1000);
  buf = 0x2D; // Z gyro, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  usleep(1000);
  bbb_readI2C(i2cd, &hibyte, 1);
  usleep(1000);
  tempint = ((short) hibyte << 8) | lobyte;
  // GYROZ_BIAS is the zero bias/offset - please adjust to make tempint close to zero for your y-gyro
  // printf("gyro z=%hd\n", tempint + GYROZ_BIAS); // With your bias this should be near zero
  gyro[2] = 0.00875*(tempint + GYROZ_BIAS);

  return;
}


void read_accel(struct I2C_data *i2cd, double accel[])
{
  short tempint;
  byte buf, lobyte, hibyte;  // Used to store I2C data
  
  buf = 0x28; // X accel, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x29; // X accel, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  accel[0] = 0.000061*tempint;    // Not sure about negative readings yet???
  
  buf = 0x2A; // Y accel, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x2B; // Y accel, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  accel[1] = 0.000061*tempint;
  
  buf = 0x2C; // Z accel, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x2D; // Z accel, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  accel[2] = 0.000061*tempint; 

  return;
}

void read_mag(struct I2C_data *i2cd, double mag[])
{
  short tempint;
  byte buf, lobyte, hibyte;  // Used to store I2C data
  
  buf = 0x08; // X mag, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x09; // X mag, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  mag[0] = 0.00008*tempint;
  
  buf = 0x0A; // Y mag, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x0B; // Y mag, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  mag[1] = 0.00008*tempint;
  
  buf = 0x0C; // Z mag, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x0D; // Z mag, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  mag[2] = 0.00008*tempint; 

  return;
}

void read_temp(struct I2C_data *i2cd, double *temp)
{
  short tempint;
  byte buf, lobyte, hibyte;  // Used to store I2C data

  buf = 0x05; // Z mag, low byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &lobyte, 1);
  buf = 0x06; // Z mag, high byte request
  bbb_writeI2C(i2cd, &buf, 1);
  bbb_readI2C(i2cd, &hibyte, 1);
  tempint = (((short) hibyte) << 8) | lobyte;
  // printf("Temp = %hd\n", tempint);
  *temp = (double) tempint; 

  return;
}
