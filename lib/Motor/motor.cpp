#include "motor.h"


uint16_t motor[4];
uint8_t PWM_PIN[4]= {20,21,22,23};

void writeMotors(){
  for( uint8_t i=0; i<4;i++){
    analogWrite(PWM_PIN[i],(motor[i]-1000)/4);
  }
}

void initMotors(){
  for(uint8_t i=0; i<4; i++){
    motor[i]=1250;
    pinMode(PWM_PIN[i],OUTPUT);
    analogWriteFrequency(PWM_PIN[i],16000);
  }
  writeMotors();
}