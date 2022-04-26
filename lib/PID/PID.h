#include "Arduino.h"

#define pidItems 4

void changePID(uint8_t mode, uint8_t param, float value);
void doPID(void);