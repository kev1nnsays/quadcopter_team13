/* DEFINES */
#define READ_TIMEOUT 1.0e-1
#define MAX_PACKET_SIZE 50

/* STANDARD VALUES WHEN CHANGING MODE */
#define JOINT_ANGLE 180.0
#define JOINT_SPEED 0.2
#define JOINT_TORQUE 0.5

#define WHEEL_SPEED 0.0
#define WHEEL_TORQUE 0.5

/* ENUMS */
enum CmdMode {JOINT, WHEEL};
enum CmdFlag {NONE, MODE, CMD, STATUS};

/*  Dynamixel Setup Struct
    Set by Dynam_Init and used for different functions
    Contains addresses and values used for communication
    NO CHANGE NECESSARY */
typedef struct{
    int fuart;
    int fgpio;
    unsigned long port;
    off_t low;
    off_t high;
    void *map_base_low,  *map_base_high;
    void *virt_addr_low, *virt_addr_high;
}DynamSetup;

/*  Dynamixel Packet Struct
    Handles the Communications Between BeagleBone and Dynamixel
    NO CHANGE NECESSARY */
typedef struct{
    unsigned char id;
    unsigned char instruction;
    unsigned char send[MAX_PACKET_SIZE-6]; /*6 diff due to header and checksum*/
    unsigned char send_size;
    unsigned char error;
    unsigned char rcvd[MAX_PACKET_SIZE];
    unsigned char rcvd_size;
}DynamPacket;

/* Servo Status Struct */
typedef struct{
    unsigned char id;
    enum CmdFlag cmd_flag;
    enum CmdMode cmd_mode;
    enum CmdMode cur_mode;
    double cmd_angle;
    double cmd_speed;
    double cmd_torque;
    double cur_angle;
    double cur_speed;
    double cur_load;
    double cur_volt;
    double cur_temp;
}DynamStatus;

/* Servo Bus Struct */
typedef struct{
    DynamStatus servo[2];
}DynamBus;

// Dynamixel Handling
int Dynam_Init(DynamSetup *dynam);
int Dynam_Comm(DynamSetup *dynam, DynamPacket *packet);
int Dynam_Deinit(DynamSetup *dynam);
int Dynam_SetMode(DynamSetup *dynam, DynamStatus *servo);
int Dynam_Command(DynamSetup *dynam, DynamStatus *servo);
int Dynam_Status(DynamSetup *dynam, DynamStatus *servo);
int Dynam_PrintStatus(DynamStatus *status);
float Dynam_PosFB(DynamStatus *status);
float Dynam_VelFB(DynamStatus *status);
int Dynam_ReadModePacket(DynamPacket *packet, DynamStatus *servo);
int Dynam_PrintPacket(DynamPacket *packet);

void * run_dynamixel_comm(void * var);
void * set_dynamixel(void * var);
