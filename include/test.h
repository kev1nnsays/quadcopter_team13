#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>
#include <pthread.h>


#include <lcm/lcm.h>
#include "../include/lcmtypes/lcmtypes_c/channels_t.h"
#include "../include/quadcopter_struct.h"
#include "../include/bbblib/bbb.h"

// Primary threads
#include "../include/run_imu.h"
#include "../include/run_motion_capture.h"
#include "../include/util.h"