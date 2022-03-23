#include "motor.h"

void writeMotors(){
  for( uint8_t i=0; i<4;i++){
    analogWrite(PWM_PIN[i],(motor[i]-1000)/4);
  }
}

void initMotors(){
  for(uint8_t i=0; i<4; i++){
    motor[i]=1000;
    pinMode(PWM_PIN[i],OUTPUT);
    analogWriteFrequency(PWM_PIN[i],38000);
  }
  writeMotors();
}