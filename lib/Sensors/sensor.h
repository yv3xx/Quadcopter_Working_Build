#include "MPU9250.h"
#include <PulsePosition.h>
#include "SensorFusion.h"

// deadband variables
#define MIDRC (1500)
#define MINRC (1000)
#define MAXRC (2000)
#define DEADBAND (35)
#define DEADTOP (MIDRC + DEADBAND)
#define DEADBOT (MIDRC - DEADBAND)





// read the ppm values and put them into our channels

void readPPM(float ppm_channels[6],PulsePositionInput myInput);


// initialize ppm
void initPPM(PulsePositionInput myInput);

void readIMU(MPU9250 IMU);
void calcIMU(MPU9250 IMU,SF fusion,float* roll, float* pitch, float* yaw);
void printAtt(float roll, float pitch, float yaw);