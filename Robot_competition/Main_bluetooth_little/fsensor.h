#ifndef FSENSOR_H_INCLUDED
#define FSENSOR_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>

void searchSensors(uint8_t *sensorsIndex); //Save in the array the address of the sensors

int getColorValue(uint8_t sn); // Get the value of the color
float getGyroValue(uint8_t sn);  // Get the value of the gyro
float getSonarValue(uint8_t sn); // Get the value of the sonar
int getCompassValue(uint8_t sn); // Get the value of the compass

#endif // FSENSOR_H_INCLUDED
