#ifndef FMOTOR_H_INCLUDED
#define FMOTOR_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include "ev3.h"

typedef struct
{
    float firstDistance;
    float lastDistance;
    float minDistance;
    float firstAngle;
    float lastAngle;
    bool detected;
} objects;

void singleSetCommand(uint8_t sn, INX_T command);
void doubleSetCommand(uint8_t sn1, uint8_t sn2, INX_T command);

void searchMotors(uint8_t *motorsIndex); //Save in the array the address of the tachos
void goStraigth(uint8_t *m, INX_T command, float speed, int time);
void turn(uint8_t *m, uint8_t *s, float degreeRight, float speedRight, int gap, float *ret);

void waitMotorEnds(uint8_t sn);
void handsCalibration(uint8_t hAdd);
bool catchBall(uint8_t *motorId,uint8_t *s, int timeDistance);
void leaveBall(uint8_t *motorId, int timeDistance);

#endif // FMOTOR_H_INCLUDED
