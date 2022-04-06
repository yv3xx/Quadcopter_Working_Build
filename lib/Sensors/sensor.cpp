#include "sensor.h"


float gx, gy, gz, ax, ay, az, mx, my, mz, temp;
float deltat;
int status2; 
SF fusion;

MPU9250 IMU(SPI,10);

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

void readIMU(){
    IMU.readSensor();
}

void calcIMU(float* roll, float* pitch, float* yaw){
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