#include "PID.h"
//#define DEBUG
// index 0 is throttle index 1 is roll index 2 is pitch index 3 is yaw
float axisCmdPID[pidItems]={0,0,0,0};
float pid_p[pidItems] = { 1.0f, 0.7f,   0.7f,   3.0f };
float pid_i[pidItems] = { 0.0f, 0.03f,  0.03f,  0.07f };
float pid_d[pidItems] = { 0.0f, 0.25f,   0.25f,   0.9f };

float prevE[pidItems] = { 0, 0, 0, 0 };
float pid_isum[pidItems] = { 0, 0, 0, 0 };
float pid_ddt[pidItems] = { 0, 0, 0, 0 };

extern float throttleCommand;
extern float rollError;
extern float pitchError;
extern float yawError;

extern float deltat;

void doPID(void){
    float e;
    // throttle
    e = throttleCommand;
    axisCmdPID[0] = (pid_p[0] * e);


    //roll
    e = rollError;
    pid_isum[1] = constrain(pid_isum[1] + (e), -0.1f, 0.1f); // constrain to stop windup
    pid_ddt[1] = (e - prevE[1]) / deltat;
    axisCmdPID[1] = (pid_p[1] * e) + (pid_i[1] * pid_isum[1]) + (pid_d[1] * pid_ddt[1]);
    axisCmdPID[1]*=-1;
    prevE[1] = e;

    //pitch
    e = pitchError;
    pid_isum[2] = constrain(pid_isum[2] + (e), -0.1f, 0.1f); // constrain to stop windup
    pid_ddt[2] = (e - prevE[2]) / deltat;
    axisCmdPID[2] = (pid_p[2] * e) + (pid_i[2] * pid_isum[2]) + (pid_d[2] * pid_ddt[2]);
    prevE[2] = e;

    //yaw
    e = yawError;
    pid_isum[3] = constrain(pid_isum[3] + (e), -0.1f, 0.1f); // constrain to stop windup
    pid_ddt[3] = (e - prevE[3]) / deltat;
    axisCmdPID[3] = (pid_p[3] * e) + (pid_i[3] * pid_isum[3]) + (pid_d[3] * pid_ddt[3]);
    prevE[3] = e;

    #ifdef DEBUG
    Serial.print("Throttle command is:");
    Serial.println(throttleCommand);
    Serial.print("Axiscmdpid0 is:");
    Serial.println(axisCmdPID[0]);
     Serial.print("Axiscmdpid1 is:");
    Serial.println(axisCmdPID[1]);
     Serial.print("Axiscmdpid2 is:");
    Serial.println(axisCmdPID[2]);
     Serial.print("Axiscmdpid3 is:");
    Serial.println(axisCmdPID[3]);
     Serial.print("deltat is:");
    Serial.println(deltat);

    #endif
}

void changePID(uint8_t mode, uint8_t param, float value){
    switch(param){
        case(1):
        pid_p[mode]=value;
        break;
        case(2):
        pid_i[mode]=value;
        break;
        case(3):
        pid_d[mode]=value;
        break;
    }
}