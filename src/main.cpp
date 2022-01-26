#include "header.h"


//#define DEBUG 
// slow loop variables
#define SLOWLOOPTARGET (1000)
unsigned long slowLoopLength;
unsigned long slowLoopStart=0;
unsigned long slowLoopTiming=0;
float ppm_channels[6];
PulsePositionInput myInput;

uint32_t start,stop;

// deadband variables
#define MIDRC (1500)
#define MINRC (1000)
#define MAXRC (2000)
#define DEADBAND (35)
#define DEADTOP (MIDRC + DEADBAND)
#define DEADBOT (MIDRC - DEADBAND)
MS5611 MS5611(0x77);

void setup() {  
  Serial.begin(9600);
  while(!Serial){
  
  } Serial.println("Poop Hi serial is done");

  
  pinMode(LED_BUILTIN,OUTPUT);
  myInput.begin(5);
  while(!myInput.available()){
  }
  Serial.println(myInput.available());

  
  if (MS5611.begin() == true)
  {
    Serial.println("MS5611 found.");
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
    while (1);
  }

  MS5611.setOversampling(OSR_ULTRA_HIGH);
  start=micros();
  int result = MS5611.read();
  stop=micros();
  if (result != MS5611_READ_OK)
  {
    Serial.print("Error in read: ");
    Serial.println(result);
  }
}

void loop() {
  uint8_t i;
  unsigned long slowLoopEnd =millis();
  slowLoopLength=slowLoopEnd- slowLoopStart;
  if((slowLoopLength)>SLOWLOOPTARGET){
    slowLoopStart=slowLoopEnd;
    long slowLoopTimingStart=micros();
    ppm_channels[5]=myInput.read(8);
    for(i=1;i<=4; i++){
      ppm_channels[i-1]=myInput.read(i);
      ppm_channels[i-1]=constrain(ppm_channels[i-1],MINRC,MAXRC);
      if((ppm_channels[i-1] > DEADBOT) && (ppm_channels[i-1] < DEADTOP)) ppm_channels[i-1]=MIDRC;
    }
    

    start=micros();
    int result = MS5611.read();
    stop=micros();
    if (result != MS5611_READ_OK)
    {
      Serial.print("Error in read: ");
      Serial.println(result);
    }
    Serial.print("T:\t");
    Serial.print(MS5611.getTemperature(), 2);
    Serial.print("\tP:\t");
    Serial.print(MS5611.getPressure(), 2);
    Serial.print("\tt:\t");
    Serial.print(stop-start);
    Serial.println();
    #ifdef DEBUG 
      Serial.print("Channel 1: "+(String)ppm_channels[0]+"Channel 2: "+(String)ppm_channels[1]+"Channel 3:"+(String)ppm_channels[2]+"Channel 4:"+(String)ppm_channels[3]+"Loop Length:"+(String)slowLoopTiming);
      Serial.println();
    #endif
    slowLoopTiming= micros() -slowLoopTimingStart;
  }
}

