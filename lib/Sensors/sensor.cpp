#include "sensor.h"
//#define DEBUG

float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float deltat;
int status2; 
SF fusion;

MPU9250 IMU(SPI,10);

//ppm stuff
float ppm_channels[6];
PPM myPPM;


// motor control things
int16_t RCrate=1.5;
int16_t RCrateyaw=3;


// motors


//ppm channels:   channel 3 is throttle
// channel 4 is yaw
//channel 1 is roll
//channrl 2 is pitch
float rollError=0;
float pitchError=0; 
float yawError=0;
const float maxAngle=45;
float halfRange=500;
float rollDiff=0;
float pitchDiff=0;

float throttleCommand=0;

const float yawRateSmooth =0.01f;
float prevYawRate=0;

float circleDiff(float prevYaw,float yaw);

void calculateErrors(float roll, float pitch, float yaw, float prevYaw){
    rollError=((float) roll - (-1* maxAngle * RCrate * ((float)((int16_t)ppm_channels[0] - (int16_t)MIDRC) / halfRange))) / 90.0f;
    //rollError*=-1;
    
    pitchError=((float) pitch - (maxAngle * RCrate * ((float)((int16_t)ppm_channels[1] - (int16_t)MIDRC) / halfRange))) / 90.0f;
    

    float yawRate=circleDiff(prevYaw,yaw)/deltat;
    yawRate=(yawRateSmooth * yawRate) + ((1.0f -yawRateSmooth)*prevYawRate);
    prevYawRate=yawRate;

    yawError=((float) yawRate - (-1* maxAngle * RCrateyaw * ((float)((int16_t)ppm_channels[3] - (int16_t)MIDRC) / halfRange))) / 90.0f;
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
    throttleCommand=ppm_channels[2]-MINRC;
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
    fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag
  //fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick

  *roll = fusion.getPitch();
  *pitch = fusion.getRoll();
  *roll+=rollDiff;
  if(*pitch>0){
      *pitch-=180;
  }
  else *pitch+=180;
  *pitch+=pitchDiff;
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
  Serial1.println("Beginning gyroscope calibration");
  status2=IMU.calibrateGyro();
  Serial1.print("Status after calibration:");
  Serial1.print(status2);
  Serial1.print("Biases are: ");
  Serial1.print(IMU.getGyroBiasX_rads());
  Serial1.print("\t");
  Serial1.print(IMU.getGyroBiasY_rads());
  Serial1.print("\t");
  Serial1.print(IMU.getGyroBiasZ_rads());
  Serial1.print("\t");
  Serial1.print("\n\n");
  IMU.setGyroBiasX_rads(IMU.getGyroBiasX_rads());
  IMU.setGyroBiasX_rads(IMU.getGyroBiasY_rads());
  IMU.setGyroBiasX_rads(IMU.getGyroBiasZ_rads());
  
  delay_NoSysTick(2000);
  Serial1.println("Beginning accelerometer calibration");
  status2=IMU.calibrateAccel();
  Serial1.print("Status after calibration:");
  Serial1.print(status2);
  Serial1.print("Biases are: ");
  Serial1.print(IMU.getAccelBiasX_mss());
  Serial1.print("\t");
  Serial1.print(IMU.getAccelBiasY_mss());
  Serial1.print("\t");
  Serial1.print(IMU.getAccelBiasZ_mss());
  Serial1.print("\t\n");
  Serial1.print("Scale Factors are: ");
  Serial1.print(IMU.getAccelScaleFactorX());
  Serial1.print("\t");
  Serial1.print(IMU.getAccelScaleFactorY());
  Serial1.print("\t");
  Serial1.print(IMU.getAccelScaleFactorZ());
  Serial1.print("\n\n");
  
  IMU.setAccelCalX(IMU.getAccelBiasX_mss(),IMU.getAccelScaleFactorX());
  IMU.setAccelCalY(IMU.getAccelBiasY_mss(),IMU.getAccelScaleFactorY());
  IMU.setAccelCalZ(IMU.getAccelBiasZ_mss(),IMU.getAccelScaleFactorZ());
  
    /*
  delay_NoSysTick(2000);
  Serial1.println("Beginning magnetometer calibration");
  status2=IMU.calibrateMag();
  Serial1.print("Status after calibration:");
  Serial1.print(status2);
  Serial1.print("Biases are: ");
  Serial1.print(IMU.getMagBiasX_uT());
  Serial1.print("\t");
  Serial1.print(IMU.getMagBiasY_uT());
  Serial1.print("\t");
  Serial1.print(IMU.getMagBiasZ_uT());
  Serial1.print("\t\n");
  Serial1.print("Scale Factors are: ");
  Serial1.print(IMU.getMagScaleFactorX());
  Serial1.print("\t");
  Serial1.print(IMU.getMagScaleFactorY());
  Serial1.print("\t");
  Serial1.print(IMU.getMagScaleFactorZ());
  Serial1.print("\n\n");
  IMU.setMagCalX(IMU.getMagBiasX_uT(),IMU.getMagScaleFactorX());
  IMU.setMagCalY(IMU.getMagBiasY_uT(),IMU.getMagScaleFactorY());
  IMU.setMagCalZ(IMU.getMagBiasZ_uT(),IMU.getMagScaleFactorZ());
  */
}
void calcDiff(float roll, float pitch){
    rollDiff=0-roll;
    pitchDiff=0-pitch;
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
/*
float calcAlt(float pressure){
    float alt=0;
}*/