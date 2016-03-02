#define EXTERN extern
#include "../include/quadcopter_main.h"
#define CRAP 999
void * set_dynamixel(void * var){


    while(1){
       float dyna1_speed = -999.9f;
       float dyna2_speed = -999.9f;
       float dyna1_pose = -999.9f;
       float dyna2_pose = -999.9f;

        switch(commandGripper){
            case 0:
                pthread_mutex_lock(&dynamixel_mutex);
                printf("Change to Joint Mode - Go to default position 0\n");
                bus.servo[0].cmd_mode = JOINT;
                bus.servo[0].cmd_flag = MODE;
                bus.servo[1].cmd_mode = JOINT;
                bus.servo[1].cmd_flag = MODE;
                pthread_mutex_unlock(&dynamixel_mutex);
                commandGripper = CRAP;
                break;
            case 1:
        		pthread_mutex_lock(&dynamixel_mutex);
        		printf("Claw open position. Moving to 0 degrees\n");
        		bus.servo[0].cmd_angle = 0.0;
        		bus.servo[0].cmd_speed = 0.5;
                bus.servo[1].cmd_angle = 0.0;
                bus.servo[1].cmd_speed = 0.5;
        		bus.servo[0].cmd_flag = CMD;
                bus.servo[1].cmd_flag = CMD;
        		pthread_mutex_unlock(&dynamixel_mutex);
                commandGripper = CRAP;
        		break;
            case 2:
                pthread_mutex_lock(&dynamixel_mutex);
                printf("Change to Wheel Mode - Stop the Servo\n");
                bus.servo[0].cmd_mode = WHEEL;
                bus.servo[0].cmd_flag = MODE;
                bus.servo[1].cmd_mode= WHEEL;
                bus.servo[1].cmd_flag = MODE;
                pthread_mutex_unlock(&dynamixel_mutex);
                commandGripper = CRAP;
                break;
            case 3:
                pthread_mutex_lock(&dynamixel_mutex);
                printf("Closing claw. Command -0.3 speed\n");
                bus.servo[0].cmd_speed = -0.3;
    		    bus.servo[0].cmd_torque = 1.0;
                bus.servo[1].cmd_speed = -0.3;
                bus.servo[1].cmd_torque = 1.0;
                bus.servo[0].cmd_flag = CMD;
                bus.servo[1].cmd_flag = CMD;
                pthread_mutex_unlock(&dynamixel_mutex);
                commandGripper = CRAP;
                break;
            case 4:
                pthread_mutex_lock(&dynamixel_mutex);
                printf("Request Status:\n");
                bus.servo[0].cmd_flag = STATUS;
                bus.servo[1].cmd_flag = STATUS;
                pthread_mutex_unlock(&dynamixel_mutex);
                float dyna1_speed = Dynam_VelFB(&bus.servo[0]);
                float dyna2_speed = Dynam_VelFB(&bus.servo[1]);
                float dyna1_pose = Dynam_PosFB(&(bus.servo[0]));
                float dyna2_pose = Dynam_PosFB(&(bus.servo[1]));
                if(dyna1_speed > 0.01 &&
                    dyna2_speed > 0.01){
                    printf("Motor Moving - Vel = (%f,%f)\n",dyna1_speed,dyna2_speed);
                    dynam_perch_signal = 0;
                }
                else if(dyna1_pose > 65 && dyna2_pose >65){
                    dynam_perch_signal = 1;
                    printf("Motor Grasped - Pose = (%f,%f)\n",dyna1_pose,dyna2_pose);
                }
                else{
                    printf("Motor Missed - Pose = (%f,%f)\n",dyna1_pose,dyna2_pose);
                    dynam_perch_signal = -1;
                }
                printf("\nServo 1 Angle: %f \n",Dynam_PosFB(&(bus.servo[0])));
                printf("\nServo 2 Angle: %f \n",Dynam_PosFB(&(bus.servo[1])));
                commandGripper = CRAP;
                break;
        }
        usleep(100000);
        fflush(stdout);

        /* If we requested Status Print Status */
        /*pthread_mutex_lock(&dynamixel_mutex);
        if(commandGripper == 4) {
            Dynam_PrintStatus(&(bus.servo[0]));
            Dynam_PrintStatus(&(bus.servo[1]));
        }
        pthread_mutex_unlock(&dynamixel_mutex);
        */
    }
    return NULL;
}
