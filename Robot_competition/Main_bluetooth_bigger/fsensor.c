#include "fsensor.h"
#include <stdio.h>
#include "ev3.h"
#include "ev3_sensor.h"

const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

void searchSensors(uint8_t *sensorsIndex) //Save in the array the address of the sensors
{
    uint8_t sn;
    //now save sensor's address
    (ev3_search_sensor( LEGO_EV3_COLOR, &sn, 0)) ? sensorsIndex[1] = sn : printf("Color sensor (2) not attached");
    (ev3_search_sensor( LEGO_EV3_GYRO, &sn, 0)) ? sensorsIndex[3] = sn : printf("Gyro sensor (4) not attached");
    (ev3_search_sensor( LEGO_EV3_US, &sn, 0)) ? sensorsIndex[2] = sn : printf("Sonar sensor (3) not attached");
    (ev3_search_sensor( HT_NXT_COMPASS, &sn, 0)) ? sensorsIndex[0] = sn : printf("Compass sensor (1) not attached");

    set_sensor_mode(sensorsIndex[1], "COL-COLOR");
    set_sensor_mode(sensorsIndex[3], "GYRO-CAL"); //reset gyro
    set_sensor_mode(sensorsIndex[3], "GYRO-ANG");
}

int getColorValue(uint8_t sn)//Return the COLOR as Array of char
{
    int val;
    if (!get_sensor_value( 0, sn, &val ) || ( val < 0 ) || ( val >= COLOR_COUNT )) val = 0;
    return val;
}

//Return the value of the GYRO
float getGyroValue(uint8_t sn)
{
    float value=-12345;
    if (!get_sensor_value0(sn, &value )) value = 0;
    return value;
}

//Return the value of the SONAR
float getSonarValue(uint8_t sn)
{
    float value=-12345;
    if (!get_sensor_value0(sn, &value )) value = 0;
    return value;
}

//Return the value of the COMPASS
int getCompassValue(uint8_t sn)
{
    float value=-12345;
    int ret;
    if (!get_sensor_value0(sn, &value )) value = 0;
    ret = (int)value;
    return ret;
}

