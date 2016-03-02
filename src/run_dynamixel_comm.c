#define EXTERN extern
#define DYNCOMM_FREQ 100 // in Hz
#include "../include/quadcopter_main.h"

void * run_dynamixel_comm(void * var){ 

    int hz = DYNCOMM_FREQ;
    
    /* Initialize ports and addresses */
    Dynam_Init(&dynam);

    /* Configure Servo Number 1 to Joint */
    pthread_mutex_lock(&dynamixel_mutex);
    bus.servo[0].id = 1;
    bus.servo[0].cmd_mode = JOINT;
    bus.servo[0].cmd_flag = MODE;
    Dynam_SetMode(&dynam,&(bus.servo[0]));

    /* Configure Servo Number 2 to Joint */
    bus.servo[1].id = 2;
    bus.servo[1].cmd_mode = JOINT;
    bus.servo[1].cmd_flag = MODE;
    Dynam_SetMode(&dynam,&(bus.servo[1]));
    pthread_mutex_unlock(&dynamixel_mutex); 

    /* Infinity Loop */
    while(1){
        pthread_mutex_lock(&dynamixel_mutex);
        /* First Servo */
        switch(bus.servo[0].cmd_flag){
            case (NONE): 
                break;
            case (CMD):
                Dynam_Command(&dynam,&(bus.servo[0]));
                break;
            case (MODE):
                Dynam_SetMode(&dynam,&(bus.servo[0]));
                break;
            case (STATUS):
                Dynam_Status(&dynam, &(bus.servo[0]));
                break;
        }
        /* Second Servo */
        switch(bus.servo[1].cmd_flag){
            case (NONE): 
                break;
            case (CMD):
                Dynam_Command(&dynam,&(bus.servo[1]));
                break;
            case (MODE):
                Dynam_SetMode(&dynam,&(bus.servo[1]));
                break;
            case (STATUS):
                Dynam_Status(&dynam, &(bus.servo[1]));
                break;
        }
        pthread_mutex_unlock(&dynamixel_mutex); 
        usleep(1000000/hz);
    }
    
    /* Deinitialize */
    Dynam_Deinit(&dynam);

}   
