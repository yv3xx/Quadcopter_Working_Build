#include <Arduino.h>
#include <PulsePosition.h>


//#define DEBUG 
// slow loop variables
#define SLOWLOOPTARGET (1000)
unsigned long slowLoopLength;
unsigned long slowLoopStart=0;
unsigned long slowLoopTiming=0;
float ppm_channels[6];
PulsePositionInput myInput;


// deadband variables
#define MIDRC (1500)
#define MINRC (1000)
#define MAXRC (2000)
#define DEADBAND (35)
#define DEADTOP (MIDRC + DEADBAND)
#define DEADBOT (MIDRC - DEADBAND)


void setup() {  
  Serial.begin(9600);
  while(!Serial){
  
  } Serial.println("Poop Hi serial is done");

  
  pinMode(LED_BUILTIN,OUTPUT);
  myInput.begin(5);
  while(!myInput.available()){
  }
  Serial.println(myInput.available());
  // put your setup code here, to run once:
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
    slowLoopTiming= micros() -slowLoopTimingStart;
    #ifdef DEBUG 
      Serial.print("Channel 1: "+(String)ppm_channels[0]+"Channel 2: "+(String)ppm_channels[1]+"Channel 3:"+(String)ppm_channels[2]+"Channel 4:"+(String)ppm_channels[3]+"Loop Length:"+(String)slowLoopTiming);
      Serial.println();
    #endif

  }
}

