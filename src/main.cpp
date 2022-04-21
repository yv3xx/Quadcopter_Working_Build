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

// state declarations for bluetooth task
enum state{
  wait,
  att,
  bat,
  alt,
  pid,
  pidType,
  pidNum
};


// fusion

float pitch, roll, yaw;
float prevYaw=0;

// timer handle for roll pitch yaw settling
TimerHandle_t settlingTimer;


SemaphoreHandle_t mutex;

volatile enum state currState;

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
  readPPM();
  //printPPM();
  readIMU();
  calcIMU(&roll,&pitch,&yaw,&prevYaw);
  //printAtt(roll,pitch,yaw);
  calculateErrors(roll,pitch,yaw,prevYaw);
  doPID();
  doMix();
  writeMotors();
  
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
    HWSERIAL.println("MS5611 found.");
  }
  else
  {
    HWSERIAL.println("MS5611 not found. halt.");
    while (1);
  }


  
  MS5611.setOversampling(OSR_STANDARD);
  int result = MS5611.read();
  if (result != MS5611_READ_OK)
  {
    HWSERIAL.print("Error in read: ");
    HWSERIAL.println(result);
  }
  HWSERIAL.println("Initializing PPM");
  initPPM();
  initIMU();
  
  calIMU();
  xTimerStart(settlingTimer,0);
  HWSERIAL.println("Starting Timer");
  while(xTimerIsTimerActive(settlingTimer)!= pdFALSE){
    readIMU();
    calcIMU(&roll,&pitch,&yaw,&prevYaw);
    //printAtt(roll,pitch,yaw);
  }
  readIMU();
  calcIMU(&roll,&pitch,&yaw,&prevYaw);
  calcDiff(roll,pitch);
  HWSERIAL.println("Hopefully the values will be the same after this!");
  xTimerDelete(settlingTimer,portMAX_DELAY);
  HWSERIAL.println("HOPEFULLY THE VLAUES WILL BE THE SAME!!");
  HWSERIAL.println("Setup is done commence flight test!");
  initMotors();
  vTaskDelete(NULL);
}

void btTask(void* parameters){
  char input;
  char buf[4];
  uint8_t i=0;
  uint8_t mode=10;
  uint8_t param=10;
  float value=0;
  while(1){
    switch(currState){
      case(wait):
        if(HWSERIAL.available()){
        input=HWSERIAL.read();
        if(i==3){
          if(strncmp(buf,cmp,3)==0){
            currState=att;
          }
          else if(strncmp(buf,cmp2,3)==0){
            currState=alt;
          }
          else if(strncmp(buf,cmp3,3)==0){
            currState=bat;
          }
          else if(strncmp(buf,cmp4,3)==0){
            HWSERIAL.println("PID command received");
            HWSERIAL.println("Would you like to adjust throttle[1] roll[2] pitch[3] or yaw[4]?");
            currState=pid;
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
        }
      break;
      case(att):
        HWSERIAL.println("Attitude command received");
        HWSERIAL.print("Attitude is: Roll:");
        HWSERIAL.print(roll);
        HWSERIAL.print(" Pitch:");
        HWSERIAL.print(pitch);
        HWSERIAL.print(" Yaw:");
        HWSERIAL.println(yaw);
        currState=wait;
      break;
      case(bat):
        HWSERIAL.println("Battery command received");
      break;
      case(alt):
        HWSERIAL.println("Altitude command received");
      break;
      case(pid):
        if(HWSERIAL.available()){
          input=HWSERIAL.read();
          switch(input){
            case('1'):
            mode=0;
            HWSERIAL.println("Would you like to adjust P[1] I[2] or D[3]?");
            HWSERIAL.read();
            currState=pidType;
            break;
            case('2'):
            mode=1;
            HWSERIAL.println("Would you like to adjust P[1] I[2] or D[3]?");
            HWSERIAL.read();
            currState=pidType;
            break;
            case('3'):
            mode=2;
            HWSERIAL.println("Would you like to adjust P[1] I[2] or D[3]?");
            HWSERIAL.read();
            currState=pidType;
            break;
            case('4'):
            mode=3;
            HWSERIAL.println("Would you like to adjust P[1] I[2] or D[3]?");
            HWSERIAL.read();
            currState=pidType;
            break;
            default:
            HWSERIAL.println("Not a valid choice!");
            break;
          }
        }
      break;
      case(pidType):
        if(HWSERIAL.available()){
          input=HWSERIAL.read();
          switch(input){
            case('1'):
            param=1;
            HWSERIAL.println("Input the value:");
            HWSERIAL.read();
            currState=pidNum;
            break;
            case('2'):
            param=2;
            HWSERIAL.println("Input the value:");
            HWSERIAL.read();
            currState=pidNum;
            break;
            case('3'):
            param=3;
            HWSERIAL.println("Input the value:");
            HWSERIAL.read();
            currState=pidNum;
            break;
            default:
            HWSERIAL.println("Not a valid choice!");
            break;
          }
        }
      break;
      case(pidNum):
      if(HWSERIAL.available()){
        value=HWSERIAL.parseFloat();
        HWSERIAL.print("Updating mode:");
        HWSERIAL.print(mode);
        HWSERIAL.print(" and parameter:");
        HWSERIAL.print(param);
        HWSERIAL.print("with value:");
        HWSERIAL.println(value);
        changePID(mode,param,value);
        currState=wait;
      }
      break;
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

  HWSERIAL.println("Poop Hi serial is done");

  settlingTimer=xTimerCreate("fusionWindup",25000/portTICK_RATE_MS,0,settlingTimer,vTimerCallback);
  mutex=xSemaphoreCreateMutex();
  xTaskCreate(setupTask, "Setup",1000,NULL,3,NULL);
  xTaskCreate(slowLoopTask,"slowLoop",1000,NULL,1,NULL);
  xTaskCreate(fastLoopTask,"fastLoop",1000,NULL,2,NULL);
  xTaskCreate(btTask,"bluetooth",400,NULL,1,NULL);
  vTaskStartScheduler();
  vTaskDelete(NULL);
}

void loop() {
  // does nothing
  vTaskDelete(NULL);
}


