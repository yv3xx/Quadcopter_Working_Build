#include "motor.h"
//#define DEBUG

uint16_t motor[4];
uint8_t PWM_PIN[4]= {20,21,22,23};
// motor 0 is front left white 1 is back left red 2 is front right white 3 is back right red
extern float throttleCorrection;
extern float axisCmdPID[];
int16_t motorCmd[4];

// index 0 is throttle index 1 is roll index 2 is pitch index 3 is yaw
void doMix(){
  for(int i=0; i<4;i++){
    motorCmd[i]=(int16_t)axisCmdPID[0]/(-1*throttleCorrection);
  }
  // mix the motors
  motorCmd[0]+= (int16_t)(axisCmdPID[0]* (-axisCmdPID[1]+axisCmdPID[2]+axisCmdPID[3]));
  motorCmd[1]+= (int16_t)(axisCmdPID[0]* (-axisCmdPID[2]-axisCmdPID[1]-axisCmdPID[3]));
  motorCmd[2]+= (int16_t)(axisCmdPID[0]* (axisCmdPID[1]+axisCmdPID[2]-axisCmdPID[3]));
  motorCmd[3]+= (int16_t)(axisCmdPID[0]* (axisCmdPID[3]-axisCmdPID[2]+axisCmdPID[1]));
  

  int16_t maxMotor=0;
  for(int i=0; i<4;i++){
    if(motorCmd[i]<0){
      motorCmd[i]=0;
    }
    if(motorCmd[i]>maxMotor)maxMotor=motorCmd[i];
  }
  float scale= 1000.0f / (float) maxMotor;
  if(maxMotor>1000){
    for(int i=0; i<4;i++){
      motorCmd[i]=(int16_t)(scale* (float)motorCmd[i]);
    }
  }
  #ifdef DEBUG
  Serial.print("motorCmd0 is:");
  Serial.println(motorCmd[0]);
  Serial.print("motorCmd1 is:");
  Serial.println(motorCmd[1]);
  Serial.print("motorCmd2 is:");
  Serial.println(motorCmd[2]);
  Serial.print("motorCmd3 is:");
  Serial.println(motorCmd[3]);
  Serial.print("throttlecorrection is:");
  Serial.println(throttleCorrection);
  #endif
  if(axisCmdPID>0){
    for(int i=0;i<4;i++){
      motor[i]=motorCmd[i]+1000;
    }
  }
}
void writeMotors(){
  for( uint8_t i=0; i<4;i++){
    analogWrite(PWM_PIN[i],(motor[i]-1000)/4);
  }
}

void initMotors(){
  for(uint8_t i=0; i<4; i++){
    motor[i]=1000;
    pinMode(PWM_PIN[i],OUTPUT);
   // analogWriteFrequency(PWM_PIN[i],16000);
  }
  writeMotors();
}