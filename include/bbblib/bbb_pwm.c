/*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *
  *  bbb_pwm.c:  PWM interface functions
  *   bbb_initPWM(byte pin):  Initialization - call once for each PWM channel used
  *
  *   bbb_setPeriodPWM(byte, unsigned int period):  Sets PWM period (in nanoseconds)
  *   bbb_setDutyPWM(byte, double percent, unsigned int period):  Sets width of PWM signal
  *   bbb_setRunStatePWM(byte, byte):  Turns PWM on and off
  *   bbb_setPolarityPWM(byte, byte):  Sets PWM polarity
  *
  *   unsigned int bbb_getPeriodPWM(byte):  Returns PWM period (in microseconds)
  *   double bbb_getDutyPWM(byte):  Returns percentage width of PWM
  *   byte bbb_getRunStatePWM(byte):  Gets PWM run state
  *   byte bbb_getPolarityPWM(byte):  Gets PWM polarity
  *
  *   PWM parameter files (one per channel):   ./period, ./duty, ./run, ./polarity
  *
  */

#define EXTERN extern
#include "bbb.h"

// PWM channel names
// Channels 0-4 correspond to SV1-5 on the breakout board
const char *pwmName[7] = { "P9_14.16", "P9_16.17", "P8_19.18", "P8_13.19", "P9_42.20" };

int bbb_initPWM(byte pin) 
{
  int returnval=0;  // Remains zero unless errors are detected
  char pwmFName[50], pwmDirName[50];

  printf("slots file path = %s\n", slotsFilePath);

  // Find directory/file for this PWM channel
  sprintf(pwmFName,"pwm_test_%s",pwmName[pin]);
  if (searchDirectory(ocpDir, pwmFName, pwmDirName)) {  // Not found
    sprintf(pwmDirName,"%s.%d", pwmFName, 15);  // 15 is a default
    returnval -= 1;
  }
 
  // pwmDir[pin]:   Full PWM directory path for channel pin
  sprintf(pwmDir[pin],"%s%s", ocpDir, pwmDirName);
  printf("dir = %s%s\n",ocpDir,pwmDirName);

  if (bbb_setPolarityPWM(pin, pwm_straight))
    returnval -= 2;

  return returnval;
}

int bbb_setPeriodPWM(byte pin, unsigned int period)
{
  int returnval=0;  // Remains zero unless errors are detected
  FILE *fp;
  char fname[50];

  if( period > 1000000000 || period < 0) {	
    printf("Error:  PWM period %d out of range.\n", period);
    returnval=-1;
  }
  sprintf(fname,"%s/period", pwmDir[pin]);
  if ((fp=fopen(fname,"w"))==NULL) {
    printf("Error opening PWM %d period file.\n",(int) pin);
    return -1;
  }
  fprintf(fp,"%u",period);
  fclose(fp);
  return 0;
}

int bbb_setDutyPWM(byte pin, double percent)
{
  FILE *fp;
  char fname[50];
  int out;
  unsigned int period = bbb_getPeriodPWM(pin);

  if( percent > 100.0 || percent < 0.0 ) {	
    printf("Error:  PWM duty percent %lf out of range.\n", percent);
    return -1;
  }
  sprintf(fname,"%s/duty", pwmDir[pin]);  
  out = (int) ((percent/100.0) * period);
  if ((fp=fopen(fname,"w"))==NULL) {
    printf("Error opening PWM %d duty cycle file.\n",(int) pin);
    return -1;
  }
  fprintf(fp,"%d",out);
  fclose(fp);
  return 0;
}

int bbb_setRunStatePWM(byte pin, byte state)
{
  FILE *fp;
  char fname[50];

  sprintf(fname,"%s/run", pwmDir[pin]);
  if ((fp=fopen(fname,"w"))==NULL) {
    printf("Error opening PWM %d run state file.\n", (int) pin);
    return -1;
  }
  fprintf(fp,"%d", (int) state);
  fclose(fp);
  return 0;
}

int bbb_setPolarityPWM(byte pin, byte polarity)
{
  FILE *fp;
  char fname[50];

  sprintf(fname,"%s/polarity", pwmDir[pin]);
  if ((fp=fopen(fname,"w"))==NULL) {
    printf("Error opening PWM %d polarity file.\n", (int) pin);
    return -1;
  }
  fprintf(fp,"%d", (int) polarity);
  fclose(fp);
  return 0;
}

unsigned int bbb_getPeriodPWM(byte pin)
{
  unsigned int period;
  FILE *fp;  char fname[50];
  sprintf(fname,"%s/period", pwmDir[pin]);
  if ((fp=fopen(fname,"r"))==NULL) {
    printf("Error opening PWM %d period file.\n", (int) pin);
    return 1000000001;
  }
  fscanf(fp,"%u", &period);  fclose(fp);
  return period;
}

double bbb_getDutyPWM(byte pin)
{
  double percent;
  unsigned int period, duty;
  period = bbb_getPeriodPWM(pin);

  FILE *fp;  char fname[50];
  sprintf(fname,"%s/duty", pwmDir[pin]);
  if ((fp=fopen(fname,"r"))==NULL) {
    printf("Error opening PWM %d duty cycle file.\n", (int) pin);
    return -100.0;
  }
  fscanf(fp,"%u", &duty);  fclose(fp);
  percent = ((double) duty)/period * 100.0;
  return percent;
}

byte bbb_getRunStatePWM(byte pin)
{
  int r;
  FILE *fp;  char fname[50];
  sprintf(fname,"%s/run", pwmDir[pin]);
  if ((fp=fopen(fname,"r"))==NULL) {
    printf("Error opening PWM %d run state file.\n", (int) pin);
    return pwm_invalid;
  }
  fscanf(fp,"%d", &r);  fclose(fp);
  if (r==0) return pwm_stop;
  else if (r==1) return pwm_run;
  else return pwm_invalid;
}

byte bbb_getPolarityPWM(byte pin)
{
  int p;
  FILE *fp;  char fname[50];
  sprintf(fname,"%s/polarity", pwmDir[pin]);
  if ((fp=fopen(fname,"r"))==NULL) {
    printf("Error opening PWM %d polarity file.\n",(int) pin);
    return pwm_invalid;
  }
  fscanf(fp,"%d", &p);  fclose(fp);
  if (p==0) return pwm_straight;
  else if (p==1) return pwm_reverse;
  else return pwm_invalid;
}
