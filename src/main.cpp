#include "header.h"


//#define DEBUG 
// slow loop variables
#define SLOWLOOPTARGET (1000)
unsigned long slowLoopLength;
unsigned long slowLoopStart=0;
unsigned long slowLoopTiming=0;
float ppm_channels[6];
PulsePositionInput myInput;

uint32_t start,stop,start1,stop1;

// deadband variables
#define MIDRC (1500)
#define MINRC (1000)
#define MAXRC (2000)
#define DEADBAND (35)
#define DEADTOP (MIDRC + DEADBAND)
#define DEADBOT (MIDRC - DEADBAND)
MS5611 MS5611(0x77);



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
  Serial.print("T:\t");
  Serial.print(MS5611.getTemperature(), 2);
  Serial.print("\tP:\t");
  Serial.print(MS5611.getPressure(), 2);
  Serial.print("\t Time:\t");
  Serial.println((stop-start));
  Serial.println();

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void fastLoopTask(void* parameters){
  uint8_t i=0;
  while(1){
  start1=micros();
  for(i=1;i<=4; i++){
    ppm_channels[i-1]=myInput.read(i);
    ppm_channels[i-1]=constrain(ppm_channels[i-1],MINRC,MAXRC);
    if((ppm_channels[i-1] > DEADBOT) && (ppm_channels[i-1] < DEADTOP)) ppm_channels[i-1]=MIDRC;
  }
  stop1=micros();
  Serial.print("\tCh 1:\t");
  Serial.print(ppm_channels[0]);
  Serial.print("\tCh 2:\t");
  Serial.print(ppm_channels[1]);
  Serial.print("\tCh 3:\t");
  Serial.print(ppm_channels[2]);
  Serial.print("\tCh 4:\t");
  Serial.print(ppm_channels[3]);
  Serial.print("\t Time:\t");
  Serial.println((stop1-start1));
  Serial.println();
  
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
 
  vTaskDelete(NULL);
}
void setup() {  
  Serial.begin(115200);

  Serial.println("Poop Hi serial is done");

  
  myInput.begin(5);
  while(!myInput.available()){
  }
  Serial.println(myInput.available());

  xTaskCreate(setupTask, "Setup",1000,NULL,3,NULL);
  xTaskCreate(slowLoopTask,"slowLoop",1000,NULL,1,NULL);
  xTaskCreate(fastLoopTask,"fastLoop",1000,NULL,2,NULL);
  vTaskStartScheduler();
  vTaskDelete(NULL);
}

void loop() {
  // does nothing
  vTaskDelete(NULL);
}


