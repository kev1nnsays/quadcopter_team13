#include "../include/quadcopter_main.h"
#include <stdio.h>

int main(){
fprintf("Servo command? (0) JointMode (1) Open (2) WheelMode (3) Close (4) Status");
int command;
fscanf("%d", &command);
set_dynamixel(command);
}


