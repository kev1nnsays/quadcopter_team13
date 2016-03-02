/*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *
  *  bbb_adc.c:  A/D converter functions
  *   bbb_initADC():  Initialize A/D
  *   bbb_readADC(adc_name ch):  Read a channel of the A/D (AIN0-AIN6)
  *
  */

#define EXTERN extern
#include "bbb.h"

int bbb_initADC() 
{
  int returnval=0;  // Remains zero unless errors are detected
  char arg1[20];

  // Load device tree
  FILE *fp_slots;
  if (!(fp_slots=fopen(slotsFilePath,"w"))) {
    printf("bbb_initADC: Error opening slots stream.\n");
    returnval -= 1;
  }
  fprintf(fp_slots,"cape-bone-iio");
  fclose(fp_slots);

  // Find ADC helper (directory) name, set ADC helper directory
  strcpy(arg1, "helper.");
  if (searchDirectory(ocpDir, arg1, ADCDirName)) {   // True if not found  
    printf("bbb_initADC():  Error finding ADC helper name.  Using helper.14\n");
    strcpy(ADCDirName,"helper.14");
    returnval -= 1;
  }
  sprintf(ADCDir,"%s%s/AIN",  ocpDir, ADCDirName);

  return returnval;  // Status (0=success)
}

int bbb_readADC(byte ch)
{
  char ADCName[100];
  FILE *fp_ADC;
  char adcstr[80];
  int adcint;

  sprintf(ADCName,"%s%d", ADCDir, (int) ch);  
  if (!(fp_ADC = fopen(ADCName, "r"))) {
    printf("getValue:  Error opening ADC Ch %d data stream.\n", (int) ch);
    return -1;
  }
  fscanf(fp_ADC,"%s", adcstr); // Maybe change this to a single fscanf of adcint (EMA)
  fclose(fp_ADC);
  adcint = atoi(adcstr);

  return adcint;  // Return value is the ADC reading in counts (int)
}
