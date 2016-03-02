#define EXTERN
#include "bbb.h"
#include "pthread.h"

int main(){

    DynamSetup dynam;
    DynamPacket packet;
    DynamBus bus;
    int i = 0;   
    int flag = 1; 

    /* Initialize ports and addresses */
    Dynam_Init(&dynam);

    bus.servo[0].id = 1;
    bus.servo[0].cmd_mode = JOINT;
    bus.servo[0].cmd_speed = 0.3;
    bus.servo[0].cmd_torque = 0.3;
    
    /* Read and Publish Status of Servo 1 */
    Dynam_ReadMode(&dynam,&(bus.servo[0]));
    printf("Mode Read!\n");
    fflush(stdout);
    Dynam_ReadStatus(&dynam,&(bus.servo[0]));
    printf("Status Read!\n");
    fflush(stdout);
    Dynam_PrintStatus(&(bus.servo[0]));
    printf("Status Printed!\n");
    fflush(stdout);
  
    /* Loop For Servo 1 */
    while(flag){
        printf("Possible Commands: \n");
        printf("1 - Change Mode\n");    
        printf("2 - Change Angle\n");    
        printf("3 - Change Speed\n");    
        printf("4 - Change Torque\n");   
        printf("5 - Check Status\n"); 
        printf("0 - Exit Program\n"); 
        printf("Your command: ");
        scanf("%d",&flag); 

        switch(flag){
            case 2:
                printf("Enter new angle: ");
                scanf("%lf", &(bus.servo[0].cmd_angle));
                Dynam_SetCmdPacket(&packet, &(bus.servo[0]));
                Dynam_Comm(&dynam, &packet);
                break;
            case 3:
                printf("Enter new speed: ");
                scanf("%lf", &(bus.servo[0].cmd_speed));
                Dynam_SetCmdPacket(&packet, &(bus.servo[0]));
                Dynam_Comm(&dynam, &packet);
                break;
            case 4:
                printf("Enter new torque: ");
                scanf("%lf", &(bus.servo[0].cmd_torque));
                Dynam_SetCmdPacket(&packet, &(bus.servo[0]));
                Dynam_Comm(&dynam, &packet);
                break;
            default:
                Dynam_ReadStatus(&dynam,&(bus.servo[0]));
                Dynam_PrintStatus(&(bus.servo[0]));
                break;
        }
    }

    /* Deinitialize */
    Dynam_Deinit(&dynam);

    return EXIT_SUCCESS;
}       
