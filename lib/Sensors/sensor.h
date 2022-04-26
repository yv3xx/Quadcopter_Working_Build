#include "MPU9250.h"
#include "PPM.h"
#include "SensorFusion.h"
#include "Arduino.h"
#include "MS5611.h"
#include <math.h>

// deadband variables
#define MIDRC (1500)
#define MINRC (1000)
#define MAXRC (2000)
#define DEADBAND (20)
#define DEADTOP (MIDRC + DEADBAND)
#define DEADBOT (MIDRC - DEADBAND)
#define PPMPIN 3

// read the ppm values and put them into our channels

void readPPM();


// initialize ppm
void initPPM();
void printPPM();
void readIMU();
void calcIMU(float* roll, float* pitch, float* yaw, float* prevYaw);
void printAtt(float roll, float pitch, float yaw);
void calIMU();
void initIMU();
void calculateErrors(float roll, float pitch, float yaw, float prevYaw);
void calcDiff(float roll, float pitch);
void initAlt();
float calcAlt();
void calcInitHeight();