#define EXTERN
#include "bbb.h"
#include "pthread.h"

/* GLOBAL VARIABLES */
DynamSetup dynam;
DynamBus bus;
pthread_mutex_t dynamixel_mutex;

void * run_dynamixel_comm(void * var){ 

    /* Initialize ports and addresses */
    Dynam_Init(&dynam);

    /* Configure Servo Number 1 to Joint */
    pthread_mutex_lock(&dynamixel_mutex);
    bus.servo[0].id = 1;
    bus.servo[0].cmd_mode = JOINT;
    Dynam_SetMode(&dynam,&(bus.servo[0]));
    pthread_mutex_unlock(&dynamixel_mutex); 

    /* Infinity Loop */
    while(1){
        pthread_mutex_lock(&dynamixel_mutex);
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
        pthread_mutex_unlock(&dynamixel_mutex); 
        usleep(50000);
    }
    
    /* Deinitialize */
    Dynam_Deinit(&dynam);

}

void * set_dynamixel(void * var){ 

    int i = 0;
    for(;i < 100;i++){        
        pthread_mutex_lock(&dynamixel_mutex);
        switch(i % 7){
            case 0:
                printf("Go To Position 10 deg with 0.4 speed\n");
                bus.servo[0].cmd_angle = 10.0; 
                bus.servo[0].cmd_speed = 0.4; 
                bus.servo[0].cmd_flag = CMD;
                break;
            case 1:
                printf("Go To Position 100 deg with 0.1 speed\n");
                bus.servo[0].cmd_angle = 100.0;
                bus.servo[0].cmd_speed = 0.1;
                bus.servo[0].cmd_flag = CMD;
                break;
            case 2:
                printf("Change to Wheel Mode - Stop the Servo\n");
                bus.servo[0].cmd_mode = WHEEL; 
                bus.servo[0].cmd_flag = MODE;
                break;
            case 3:     
                printf("Command 0.3 speed\n");
                bus.servo[0].cmd_speed = 0.3; 
                bus.servo[0].cmd_flag = CMD;
                break;
            case 4:
                printf("Request Status:\n");
                bus.servo[0].cmd_flag = STATUS;
                break;
            case 5:     
                printf("Command -0.3 speed\n");
                bus.servo[0].cmd_speed = -0.3; 
                bus.servo[0].cmd_flag = CMD;
                break;
            case 6:
                printf("Change to Joint Mode - Go to default position\n");
                bus.servo[0].cmd_mode = JOINT; 
                bus.servo[0].cmd_flag = MODE;
                break;
        }
        fflush(stdout);
        pthread_mutex_unlock(&dynamixel_mutex);
        sleep(5);

        /* If we requested Status Print Status */
        pthread_mutex_lock(&dynamixel_mutex);
        if(i%7 == 4) Dynam_PrintStatus(&(bus.servo[0]));
        pthread_mutex_unlock(&dynamixel_mutex);
    }
}
         
int main(){

    pthread_t dynamixel_comm_thread;
    pthread_t dynamixel_set_thread;

    pthread_mutex_init(&dynamixel_mutex, NULL);    

    pthread_create(&dynamixel_comm_thread, NULL, run_dynamixel_comm, NULL);
    sleep(5);
    pthread_create(&dynamixel_set_thread, NULL, set_dynamixel, NULL);

    pthread_join(dynamixel_comm_thread, NULL);
    pthread_join(dynamixel_set_thread, NULL);

    return EXIT_SUCCESS;
}       
