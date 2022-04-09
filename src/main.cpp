#include "header.h"


//#define DEBUG 


uint32_t start,stop,start1,stop1;

// barometer +temp
MS5611 MS5611(0x77);

// bluetooth
#define HWSERIAL Serial1

// compares for bluetooth commands
static const char cmp[4]="att";
static const char cmp2[4]="alt";
static const char cmp3[4]="bat";
static const char cmp4[4]="pid";

// mpu

int status;




// fusion

float pitch, roll, yaw;

// timer handle for roll pitch yaw settling
TimerHandle_t settlingTimer;


SemaphoreHandle_t mutex;

void vTimerCallback(TimerHandle_t xTimer){
}

void slowLoopTask(void* parameters){
  while(1){
  start=micros();
  
  int result = MS5611.read();
  if (result != MS5611_READ_OK)
    {
      Serial.print("Error in read: ");
      Serial.println(result);
    }
  readPPM();
  printPPM();
  stop=micros();
  #ifdef DEBUG
    Serial.print("T:\t");
    Serial.print(MS5611.getTemperature(), 2);
    Serial.print("\tP:\t");
    Serial.print(MS5611.getPressure(), 2);
    Serial.print("\t Time:\t");
    Serial.println((stop-start));
    Serial.println();
  #endif

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void fastLoopTask(void* parameters){
  while(1){
  start1=micros();
  //readPPM(ppm_channels,myInput);
  readIMU();
  calcIMU(&roll,&pitch,&yaw);
 // printAtt(roll,pitch,yaw);
 // writeMotors();
  
  stop1=micros();
  #ifdef DEBUG
    Serial.print("\tCh 1:\t");
    Serial.print(ppm_channels[0]);
    Serial.print("\tCh 2:\t");
    Serial.print(ppm_channels[1]);
    Serial.print("\tCh 3:\t");
    Serial.print(ppm_channels[2]);
    Serial.print("\tCh 4:\t");
    Serial.print(ppm_channels[3]);
    
    Serial.print(IMU.getAccelX_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(),6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(),6);
    Serial.println("\t");
    Serial.print("\t Time:\t");
    Serial.println((stop1-start1));
    Serial.println();
  #endif
  vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}
void setupTask(void* parameters){

  

  if (MS5611.begin() == true)
  {
    Serial.println("MS5611 found.");
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
    while (1);
  }


  
  MS5611.setOversampling(OSR_STANDARD);
  int result = MS5611.read();
  if (result != MS5611_READ_OK)
  {
    Serial.print("Error in read: ");
    Serial.println(result);
  }
  Serial.println("Initializing PPM");
  initPPM();
  initIMU();
  
  xTimerStart(settlingTimer,0);
  Serial.println("Starting Timer");
  while(xTimerIsTimerActive(settlingTimer)!= pdFALSE){
    readIMU();
    calcIMU(&roll,&pitch,&yaw);
    //printAtt(roll,pitch,yaw);
  }

  Serial.println("Hopefully the values will be the same after this!");
  xTimerDelete(settlingTimer,portMAX_DELAY);
  Serial.println("HOPEFULLY THE VLAUES WILL BE THE SAME!!");
  
  initMotors();
  vTaskDelete(NULL);
}

void btTask(void* parameters){
  char input;
  char buf[4];
  uint8_t i=0;
  while(1){
    if(HWSERIAL.available()){
      input=HWSERIAL.read();
      if(i==3){
        if(strncmp(buf,cmp,3)==0){
          Serial.println("attitude command received");
        }
        else if(strncmp(buf,cmp2,3)==0){
          Serial.println("altitude command received");
        }
        else if(strncmp(buf,cmp3,3)==0){
          Serial.println("battery command received");
        }
        else if(strncmp(buf,cmp4,3)==0){
          Serial.println("pid command received");
        }
          memset(buf,0,4);
          i=0;
      }
      buf[i]=input;
      i++;
      if(input== '\n'){
        memset(buf,0,4);
        i=0;
      }
      Serial.print(input);
      
    }
    if(Serial.available()){
      input=Serial.read();
      Serial.print(input);
      HWSERIAL.print(input);
    }
  }
}
void setup() {  
  Serial.begin(115200);
  HWSERIAL.begin(115200);

  Serial.println("Poop Hi serial is done");

  settlingTimer=xTimerCreate("fusionWindup",23000/portTICK_RATE_MS,0,settlingTimer,vTimerCallback);
  mutex=xSemaphoreCreateMutex();
  xTaskCreate(setupTask, "Setup",1000,NULL,3,NULL);
  xTaskCreate(slowLoopTask,"slowLoop",1000,NULL,1,NULL);
  xTaskCreate(fastLoopTask,"fastLoop",1000,NULL,2,NULL);
  xTaskCreate(btTask,"bluetooth",250,NULL,1,NULL);
  vTaskStartScheduler();
  vTaskDelete(NULL);
}

void loop() {
  // does nothing
  vTaskDelete(NULL);
}


