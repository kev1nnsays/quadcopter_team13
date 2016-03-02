/*
  *  Beaglebone Black Hardware Interface Function Library
  *  For Use in Robotics 550, University of Michigan
  *
  *  References:  
  *      http://beagleboard.org/
  *      BlackLib: http://free.com/projects/blacklib
  *
  *  bbb_init.c:  Initialization functions
  *
  */

#define EXTERN extern
#include "bbb.h"

int bbb_init()  // Overall initialization of names
{
  int returnval=0, i;  // returnval remains zero unless errors are detected
  char arg1[50], arg2[50];

  // Initialize cape manager name
  strcpy(arg1, "/sys/devices/");  strcpy(arg2, "bone_capemgr.");
  if (searchDirectory(arg1, arg2, capeMgrName)) {     // Not found  
    printf("bbb_init():  Error finding cape manager name.  Using bone_capemgr.8.\n");
    strcpy(capeMgrName,"bone_capemgr.8");
    returnval -= 1;
  }

  // Initialize ocp name
  strcpy(arg2, "ocp.");
  if (searchDirectory(arg1, arg2, ocpName)) {   // Not found  
    printf("bbb_init():  Error finding ocp name.  Using ocp.2.\n");
    strcpy(ocpName,"ocp.2");
    returnval -= 1;
  }
  sprintf(ocpDir,"/sys/devices/%s/",ocpName);

  // Set slots path 
  sprintf(slotsFilePath,"/sys/devices/%s/slots",capeMgrName);

  // Initialize GPIO status flags to invalid (not ready for read or write)
  for (i=0;i<126;i++) gpio_stat[i] = gpio_invalid; 

  return returnval;
}

int searchDirectory(char *devdir, char *searchstr, char *returnstr)
{
  DIR *path;
  struct dirent *entry;  
  path = opendir(devdir);
  if(path != NULL) {
    while( (entry = readdir(path)) != NULL) {
      if( entry->d_name[0] == '.')
	continue;
      if(strstr(entry->d_name,searchstr) != NULL ) {
	closedir(path);
        strcpy(returnstr, entry->d_name);
	return 0;  // Found
      }
    }
  }
  closedir(path);  
  return -1;  // Not found
}

