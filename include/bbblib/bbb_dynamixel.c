#define EXTERN extern

#include "bbb.h"
#include <sys/time.h>

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

/*  #####################################################################
    #                    INITIALIZE DYNAMIXEL                           #
    #####################################################################*/
int Dynam_Init(DynamSetup *dynam){

    /* Initialize GPIO */
    bbb_init();
    bbb_initGPIO(61,gpio_output);

    /* Initialize UART Port */
    dynam->fuart = initUART();

    /* GPIO Port Address */
    dynam->port = 0x20000000;

    /* GPIO 1 Command Low and High*/
    dynam->low  = 0x4804c190;
    dynam->high = 0x4804c194;

    /* Open File */
    if((dynam->fgpio = open("/dev/mem", O_RDWR | O_SYNC)) == -1) return EXIT_FAILURE;

    /* Map Low */
    dynam->map_base_low = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                                     dynam->fgpio, dynam->low & ~MAP_MASK);
    if(dynam->map_base_low == (void *) -1) return EXIT_FAILURE;
    dynam->virt_addr_low = dynam->map_base_low + (dynam->low & MAP_MASK);

    /* Map High */
    dynam->map_base_high = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                                     dynam->fgpio, dynam->high & ~MAP_MASK);
    if(dynam->map_base_high == (void *) -1) return EXIT_FAILURE;
    dynam->virt_addr_high = dynam->map_base_high + (dynam->high & MAP_MASK);

    return EXIT_SUCCESS;
}

/*  #####################################################################
    #               COMMUNICATES WITH DYNAMIXEL                         #
    #####################################################################*/
int Dynam_Comm(DynamSetup *dynam, DynamPacket *packet){

    int i = 0;
    unsigned char data[MAX_PACKET_SIZE];
    unsigned char checksum;
    struct timeval start, now;

    /* Prepare data to send */
    data[0] = 0xFF; /* Dinamixel Header */
    data[1] = 0xFF; /* Dinamixel Header */
    data[2] = packet->id;   /* Servo ID*/
    checksum = packet->id;   /* Initialize Checksum */
    data[3] = (packet->send_size + 2); /* Package Lenght */
    checksum += data[3];
    data[4] = packet->instruction;
    checksum += data[4];
    for(i = 0; i < packet->send_size; i++){
        data[i+5] = packet->send[i];
        checksum += data[i+5];
    }
    checksum = ~(checksum);
    data[i+5] = checksum;

    /* Command Low */
    *((unsigned long *) dynam->virt_addr_low) = dynam->port;

    /* Write data */
    write(dynam->fuart, data, packet->send_size + 6);
    tcdrain(dynam->fuart);

    /* Command High */
    *((unsigned long *) dynam->virt_addr_high) = dynam->port;

    /* Read data start */
    gettimeofday(&start,NULL);
    checksum = 0;

    /* Read Header Bytes */
    for(i = 0; i < 2; i++){

        /* Wait for First Data with Timeout*/
        while(read(dynam->fuart, &(data[i]), 1) < 1){
            gettimeofday(&now,NULL);
            if(now.tv_sec - start.tv_sec + 1e-6*(now.tv_usec - start.tv_usec)
                > READ_TIMEOUT){
                printf("ERROR - Timeout of Read Serial - header\n");
                return EXIT_FAILURE;
            }
        }

        /* Check correct header byte */
        if (data[i] != 0xFF){
      //      printf("Incorrect Header - 0x%x\n",data[i]);
            i--;
        }

    }
    //fflush(stdout);

    /* Read ID */
    while(read(dynam->fuart, &(data[2]), 1) < 1){
        gettimeofday(&now,NULL);
        if(now.tv_sec - start.tv_sec + 1e-6*(now.tv_usec - start.tv_usec)
            > READ_TIMEOUT){
            printf("ERROR - Timeout of Read Serial - id\n");
            return EXIT_FAILURE;
        }
    }
    if(data[2] != packet->id){
        printf("ERROR - Received ID Byte Wrong\n");
        return EXIT_FAILURE;
    }
    checksum = packet->id;

    /* Read Length */
    while(read(dynam->fuart, &(packet->rcvd_size), 1) < 1){
        gettimeofday(&now,NULL);
        if(now.tv_sec - start.tv_sec + 1e-6*(now.tv_usec - start.tv_usec)
            > READ_TIMEOUT){
            printf("ERROR - Timeout of Read Serial - length\n");
            return EXIT_FAILURE;
        }
    }
    checksum += packet->rcvd_size;
    packet->rcvd_size -= 2;

    /* Read Error */
    while(read(dynam->fuart, &(packet->error), 1) < 1){
        gettimeofday(&now,NULL);
        if(now.tv_sec - start.tv_sec + 1e-6*(now.tv_usec - start.tv_usec)
            > READ_TIMEOUT){
            printf("ERROR - Timeout of Read Serial - error\n");
            return EXIT_FAILURE;
        }
    }
    if(packet->error != 0){
        printf("ERROR - Servo Status Returned Error\n");
        return EXIT_FAILURE;
    }
    checksum += packet->error;

    /* Read Parameters */
    for(i = 0; i < packet->rcvd_size; i++){
        while(read(dynam->fuart, &(packet->rcvd[i]), 1) < 1){
            gettimeofday(&now,NULL);
            if(now.tv_sec - start.tv_sec + 1e-6*(now.tv_usec - start.tv_usec)
                > READ_TIMEOUT){
                printf("ERROR - Timeout of Read Serial - packet\n");
                return EXIT_FAILURE;
            }
        }
        //printf("%u \n",packet->rcvd[i]);
        checksum += packet->rcvd[i];
    }

    /* Read Checksum */
    while(read(dynam->fuart, &(data[0]), 1) < 1){
            gettimeofday(&now,NULL);
            if(now.tv_sec - start.tv_sec + 1e-6*(now.tv_usec - start.tv_usec)
                > READ_TIMEOUT){
                printf("ERROR - Timeout of Read Serial - checksum\n");
                return EXIT_FAILURE;
            }
    }
    checksum = ~(checksum);
    if(data[0] != checksum){
        printf("ERROR - Received Checksum Error\n");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/*  #####################################################################
    #                  DE-INITIALIZE  DYNAMIXEL                         #
    #####################################################################*/
int Dynam_Deinit(DynamSetup *dynam){

    /* Deinitialize GPIO */
    bbb_deinitGPIO(61);

    /* Close UART */
    closeUART(dynam->fuart);

    /* Unmap both low and high */
    if(munmap(dynam->map_base_low, MAP_SIZE) == -1) return EXIT_FAILURE;
    if(munmap(dynam->map_base_high, MAP_SIZE) == -1) return EXIT_FAILURE;

    /* Close GPIO Map File */
    close(dynam->fgpio);

    return EXIT_SUCCESS;

}

/*  #####################################################################
    #                         SET MODE FUNCTION                         #
    #####################################################################*/
int Dynam_SetMode(DynamSetup *dynam, DynamStatus *servo){

    DynamPacket packet;

    /* Prepare Packet to be Sent */
    packet.id = servo->id;
    packet.instruction = 0x03;
    packet.send[0] = 0x06;
    if(servo->cmd_mode == JOINT){
        packet.send[1] = 0x0;
        packet.send[2] = 0x0;
        packet.send[3] = 0xFF;
        packet.send[4] = 0x03;
    }
    else{
        packet.send[1] = 0x0;
        packet.send[2] = 0x0;
        packet.send[3] = 0x0;
        packet.send[4] = 0x0;
    }
    packet.send_size = 5;

    /* Perform the Communication */
    if(Dynam_Comm(dynam, &packet)){
        printf("Error Changing Mode for Servo ID : %d\n",servo->id);
        return EXIT_FAILURE;
    }
    else servo->cmd_flag = NONE;

    /* First Set the Speed to 0 if going to Wheel Mode
       or 0.2 if going to JOINT Mode
       Behavior is strange if you change mode with speed */
    if(servo->cmd_mode == JOINT){
        servo->cmd_angle = JOINT_ANGLE;
        servo->cmd_speed = JOINT_SPEED;
        servo->cmd_torque = JOINT_TORQUE;
    }
    else{
        servo->cmd_speed = WHEEL_SPEED;
        servo->cmd_torque = WHEEL_TORQUE;
    }

    /* Perform the Communication */
    if(Dynam_Command(dynam,servo)){
        servo->cmd_flag = CMD;
        printf("Error Changing Mode (STD CMD) for Servo ID : %d\n",servo->id);
        return EXIT_FAILURE;
    }
    else servo->cmd_flag = NONE;

    return EXIT_SUCCESS;
}

/*  #####################################################################
    #                     COMMAND SERVO FUNCTION                        #
    #####################################################################*/
int Dynam_Command(DynamSetup *dynam, DynamStatus *servo){

    DynamPacket packet;

    /* Prepare the Packet */
    packet.id = servo->id;
    packet.instruction = 0x03;
    uint16_t aux = 0;

    /* Joint Mode */
    if(servo->cmd_mode == JOINT){
        packet.send[0] = 0x1E;

        /* Desired Position */
        aux = (uint16_t) (servo->cmd_angle / 0.29);
        packet.send[1] = aux & 0xFF;
        packet.send[2] = (aux >> 8) & 0xFF;

        /* Desired Speed */
        if(servo->cmd_speed > 1.0) servo->cmd_speed = 1.0;
        if(servo->cmd_speed < 0.0) servo->cmd_speed = 0.0;
        aux = (uint16_t) (servo->cmd_speed * 1023);
        packet.send[3] = aux & 0xFF;
        packet.send[4] = (aux >> 8) & 0xFF;

        /* Desired Torque */
        if(servo->cmd_torque > 1.0) servo->cmd_torque = 1.0;
        if(servo->cmd_torque < 0.0) servo->cmd_torque = 0.0;
        aux = (uint16_t) (servo->cmd_torque * 1023);
        packet.send[5] = aux & 0xFF;
        packet.send[6] = (aux >> 8) & 0xFF;

        packet.send_size = 7;
    }
    /* Wheel Mode */
    else if (servo->cmd_mode == WHEEL){
        packet.send[0] = 0x20;

        /* Desired Speed */
        if(fabs(servo->cmd_speed) > 1.0)
            aux = 1023;
        else
            aux = (unsigned int) (fabs(servo->cmd_speed) *1023);
        if(servo->cmd_speed < 0.0)
            aux = aux | 0x400;
        packet.send[1] = aux & 0xFF;
        packet.send[2] = (aux >> 8) & 0xFF;

        /* Desired Torque */
        if(servo->cmd_torque > 1.0) servo->cmd_torque = 1.0;
        if(servo->cmd_torque < 0.0) servo->cmd_torque = 0.0;
        aux = (unsigned int) (servo->cmd_torque * 1023);
        packet.send[3] = aux & 0xFF;
        packet.send[4] = (aux >> 8) & 0xFF;

        packet.send_size = 5;
    }
    else
        return EXIT_FAILURE;

    /* Perform the Communication */
    if(Dynam_Comm(dynam, &packet)){
        printf("Error Changing Command for Servo ID : %d\n",servo->id);
        return EXIT_FAILURE;
    }
    else servo->cmd_flag = NONE;

    return EXIT_SUCCESS;
}

/*  #####################################################################
    #                     READ SERVO STATUS                             #
    #####################################################################*/
int Dynam_Status(DynamSetup *dynam, DynamStatus *servo){

    DynamPacket packet;
    uint16_t aux;

    packet.id = servo->id;
    packet.instruction = 0x02;
    packet.send[0] = 0x24;
    packet.send[1] = 0x08;
    packet.send_size = 2;

    if(Dynam_Comm(dynam, &packet)){
        printf("Error Reading Status of Servo ID : %d\n",servo->id);
        return EXIT_FAILURE;
    }
    servo->cmd_flag = NONE;

    /* Read Current Angle */
    aux = (packet.rcvd[1] << 8) | packet.rcvd[0];
    servo->cur_angle = ((double) aux)*0.29;

    /* Read Current Speed */
    aux = (packet.rcvd[3] << 8) | packet.rcvd[2];
    servo->cur_speed = ((double) aux) / 1023.0;

    /* Read Current Load */
    aux = (packet.rcvd[5] << 8) | packet.rcvd[4];
    servo->cur_load = ((double) aux) / 1023.0;

    /* Read Voltage */
    servo->cur_volt = packet.rcvd[6] / 10.0;

    /* Read Voltage */
    servo->cur_temp = packet.rcvd[7];

    return EXIT_SUCCESS;
}

/*  #####################################################################
    #                     PRINT SERVO STATUS                             #
    #####################################################################*/
int Dynam_PrintStatus(DynamStatus *status){

    printf("\n\t Status For Servo ID #%d\n\n",status->id);
    if(status->cmd_mode)   printf("Command Mode: \t WHEEL \n");
    else           printf("Command Mode: \t JOINT \n");

    printf(" COMMANDED Values:\n");
    if(!(status->cmd_mode))   printf(" Angle:       %.1lf\n",status->cmd_angle);
    printf(" Speed:       %.1lf\n",status->cmd_speed);
    printf(" Torque:      %.1lf\n\n",status->cmd_torque);

    printf(" CURRENT Values:\n");
    printf(" Angle:       %.1lf\n",status->cur_angle);
    printf(" Speed:       %.1lf\n",status->cur_speed);
    printf(" Load:        %.1lf\n",status->cur_load);
    printf(" Voltage:     %.1lf\n",status->cur_volt);
    printf(" Temperatue:  %.1lf\n\n",status->cur_temp);

    fflush(stdout);

    return EXIT_SUCCESS;
}

float Dynam_PosFB(DynamStatus *status){
    return status->cur_angle;
    }
float Dynam_VelFB(DynamStatus *status){
    return status->cur_speed;
    }


/*  #####################################################################
    #                     OTHER FUNCTIONS NOT USED                      #
    #####################################################################*/
int Dynam_ReadModePacket(DynamPacket *packet, DynamStatus *servo){

    if(packet->send[0] != 0x06){
        printf("ERROR - Attempting to Read Wrong Packet\n");
        return EXIT_FAILURE;
    }

    unsigned int aux;
    float min_angle, max_angle;

    servo->cur_mode = WHEEL;
    aux = (packet->rcvd[1] << 8) | packet->rcvd[0];
    if(aux) servo->cur_mode = JOINT;
    min_angle = aux * 0.29;
    aux = (packet->rcvd[3] << 8) | packet->rcvd[2];
    max_angle = aux * 0.29;
    if(aux) servo->cur_mode = JOINT;

    return EXIT_SUCCESS;
}


int Dynam_PrintPacket(DynamPacket *packet){

    int i = 0;

    printf("ID:           0x%02x\n", packet->id);
    printf("Instruction:  0x%02x\n", packet->instruction);

    for(i = 0; i < packet->send_size; i++)
        printf("Send %d Byte:  0x%02x\n", i, packet->send[i]);

    printf("Error:  0x%02x\n", packet->error);

    for(i = 0; i < packet->rcvd_size; i++)
        printf("Rcvd %d Byte:  0x%02x\n", i, packet->rcvd[i]);

    return EXIT_SUCCESS;

}


