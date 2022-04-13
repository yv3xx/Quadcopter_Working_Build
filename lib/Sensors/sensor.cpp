#include "sensor.h"
#define DEBUG

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float deltat;
int status2; 
SF fusion;

MPU9250 IMU(SPI,10);

//ppm stuff
float ppm_channels[6];
PPM myPPM;


// motor control things
int16_t RCrate=2;


// motors


//ppm channels:   channel 3 is throttle
// channel 4 is yaw
//channel 1 is roll
//channrl 2 is pitch
extern float throttleCorrection;
float rollError=0;
float pitchError=0; 
float yawError=0;
const float maxAngle=45;
float halfRange=500;

float throttleCommand=0;

const float yawRateSmooth =0.01f;
float prevYawRate=0;

float circleDiff(float prevYaw,float yaw);

void calculateErrors(float roll, float pitch, float yaw, float prevYaw){
    rollError=((float) roll - (maxAngle * RCrate * ((float)((int16_t)ppm_channels[0] - (int16_t)MIDRC) / halfRange))) / 90.0f;
    
    pitchError=((float) pitch - (maxAngle * RCrate * ((float)((int16_t)ppm_channels[1] - (int16_t)MIDRC) / halfRange))) / 90.0f;
    

    float yawRate=circleDiff(prevYaw,yaw)/.002;
    yawRate=(yawRateSmooth * yawRate) + ((1.0f -yawRateSmooth)*prevYawRate);
    prevYawRate=yawRate;

    yawError=((float) yawRate - (maxAngle * RCrate * ((float)((int16_t)ppm_channels[3] - (int16_t)MIDRC) / halfRange))) / 90.0f;
    #ifdef DEBUG
        Serial.print("Yaw is:");
        Serial.println(yaw);
        Serial.print("prevYaw is:");
        Serial.println(prevYaw);
        Serial.print("circlediff is:");
        Serial.println(circleDiff(prevYaw,yaw));
        Serial.print("yawRate is:");
        Serial.println(yawRate);
        Serial.print("Roll error is:");
        Serial.println(rollError);
        Serial.print("Pitch error is:");
        Serial.println(pitchError);
        Serial.print("YawRate error is:");
        Serial.println(yawError);
    #endif
    throttleCommand=ppm_channels[2];
}

void readPPM(){
    uint8_t i=0;
    for(i=1;i<=4; i++){
        ppm_channels[i-1]=myPPM.dataRead(PPMPIN,i);
        ppm_channels[i-1]=constrain(ppm_channels[i-1],MINRC,MAXRC);
        if((ppm_channels[i-1] > DEADBOT) && (ppm_channels[i-1] < DEADTOP)) ppm_channels[i-1]=MIDRC;
    }
}

void initPPM(){
    myPPM.PPMInput(FALLING,1,PPMPIN);
    while(!myPPM.dataAvl(PPMPIN)){
    }
}

void printPPM(){
    Serial.print("\tCh 1:\t");
    Serial.print(ppm_channels[0]);
    Serial.print("\tCh 2:\t");
    Serial.print(ppm_channels[1]);
    Serial.print("\tCh 3:\t");
    Serial.print(ppm_channels[2]);
    Serial.print("\tCh 4:\t");
    Serial.print(ppm_channels[3]);
    Serial.println();
}
void readIMU(){
    IMU.readSensor();
}

void calcIMU(float* roll, float* pitch, float* yaw,float* prevYaw){
    ax = IMU.getAccelX_mss();
    ay = IMU.getAccelY_mss();
    az = IMU.getAccelZ_mss();
    gx = IMU.getGyroX_rads();
    gy = IMU.getGyroY_rads();
    gz = IMU.getGyroZ_rads();
    mx = IMU.getMagX_uT();
    my = IMU.getMagY_uT();
    mz = IMU.getMagZ_uT();
    temp = IMU.getTemperature_C();

    deltat = fusion.deltatUpdate();
 // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick

  *roll = fusion.getRoll();
  if(*roll>0){
      *roll-=180;
  }
  else *roll+=180;
  *pitch = fusion.getPitch();
  *prevYaw=*yaw;
  *yaw = fusion.getYaw();
}


void printAtt(float roll, float pitch, float yaw){
  Serial.print("Roll= ");
  Serial.print(roll);
  Serial.print(", Pitch = ");
  Serial.print(pitch);
  Serial.print(", Yaw = ");
  Serial.println(yaw);
}

void calIMU(){

  Serial.println("Beginning gyroscope calibration");
  status2=IMU.calibrateGyro();
  Serial.print("Status after calibration:");
  Serial.print(status2);
  Serial.print("Biases are: ");
  Serial.print(IMU.getGyroBiasX_rads());
  Serial.print("\t");
  Serial.print(IMU.getGyroBiasY_rads());
  Serial.print("\t");
  Serial.print(IMU.getGyroBiasZ_rads());
  Serial.print("\t");
  Serial.print("\n\n");

  
  delay_NoSysTick(2000);
  Serial.println("Beginning accelerometer calibration");
  status2=IMU.calibrateAccel();
  Serial.print("Status after calibration:");
  Serial.print(status2);
  Serial.print("Biases are: ");
  Serial.print(IMU.getAccelBiasX_mss());
  Serial.print("\t");
  Serial.print(IMU.getAccelBiasY_mss());
  Serial.print("\t");
  Serial.print(IMU.getAccelBiasZ_mss());
  Serial.print("\t\n");
  Serial.print("Scale Factors are: ");
  Serial.print(IMU.getAccelScaleFactorX());
  Serial.print("\t");
  Serial.print(IMU.getAccelScaleFactorY());
  Serial.print("\t");
  Serial.print(IMU.getAccelScaleFactorZ());
  Serial.print("\n\n");
  
  delay_NoSysTick(2000);
  Serial.println("Beginning magnetometer calibration");
  status2=IMU.calibrateMag();
  Serial.print("Status after calibration:");
  Serial.print(status2);
  Serial.print("Biases are: ");
  Serial.print(IMU.getMagBiasX_uT());
  Serial.print("\t");
  Serial.print(IMU.getMagBiasY_uT());
  Serial.print("\t");
  Serial.print(IMU.getMagBiasZ_uT());
  Serial.print("\t\n");
  Serial.print("Scale Factors are: ");
  Serial.print(IMU.getMagScaleFactorX());
  Serial.print("\t");
  Serial.print(IMU.getMagScaleFactorY());
  Serial.print("\t");
  Serial.print(IMU.getMagScaleFactorZ());
  Serial.print("\n\n");
 
}

void initIMU(){
    status2 = IMU.begin();
    if (status2 < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status2);
        while(1) {}
    }
    IMU.setGyroBiasX_rads(-.02f);
    IMU.setGyroBiasY_rads(0.0f);
    IMU.setGyroBiasZ_rads(.02f);

    IMU.setMagCalX(14.89f,1.136f);
    IMU.setMagCalY(-11.466f,1.163f);
    IMU.setMagCalZ(-69.6f,.81f);
    
    Serial.println("IMU intialization successful");

}

float circleDiff(float prevYaw,float yaw){
    float diff=0;
    if((prevYaw >315) && (yaw<45)){
        diff= yaw + (360-prevYaw);
    }
    else if((yaw>315) && (prevYaw<45)){
        diff=(prevYaw + (360-yaw))*-1;
    } else {
        diff= yaw - prevYaw;
    }
    return(diff);
}