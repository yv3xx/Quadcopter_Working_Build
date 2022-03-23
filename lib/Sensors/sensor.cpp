#include "sensor.h"


float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float deltat;

void readPPM(float ppm_channels[6],PulsePositionInput myInput){
    uint8_t i=0;
    for(i=1;i<=4; i++){
        ppm_channels[i-1]=myInput.read(i);
        ppm_channels[i-1]=constrain(ppm_channels[i-1],MINRC,MAXRC);
        if((ppm_channels[i-1] > DEADBOT) && (ppm_channels[i-1] < DEADTOP)) ppm_channels[i-1]=MIDRC;
    }
}

void initPPM(PulsePositionInput myInput){
    myInput.begin(5);
    while(!myInput.available()){
    }
}

void readIMU(MPU9250 IMU){
    IMU.readSensor();
}

void calcIMU(MPU9250 IMU,SF fusion,float* roll, float* pitch, float* yaw){
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
  //fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //mahony is suggested if there isn't the mag
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick

  *roll = fusion.getRoll();
  *pitch = fusion.getPitch();
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