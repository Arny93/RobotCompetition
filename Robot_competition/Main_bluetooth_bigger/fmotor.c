#include "fmotor.h"
#include "ev3_tacho.h"
#include "fsensor.h"
#include <stdio.h>
#include <unistd.h>

#ifndef _sleepFunction
#define _sleepFunction
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif // _sleepFunction

void singleSetCommand(uint8_t sn, INX_T command)
{
    set_tacho_command_inx(sn, command);
    Sleep(100);
}

void doubleSetCommand(uint8_t sn1, uint8_t sn2, INX_T command)
{
    set_tacho_command_inx(sn1, command);
    set_tacho_command_inx(sn2, command);
    Sleep(100);
}

void searchMotors(uint8_t *motorsIndex)
{
    uint8_t sn;
    ( ev3_search_tacho_plugged_in( 66, 0, &sn, 0)) ? motorsIndex[0] = sn : printf("Motor on port B not attached");
    ( ev3_search_tacho_plugged_in( 67, 0, &sn, 0)) ? motorsIndex[1] = sn : printf("Motor on port C not attached");
    ( ev3_search_tacho_plugged_in( 65, 0, &sn, 0)) ? motorsIndex[2] = sn : printf("Motor on port A not attached");
    //Reset all the motors
    multi_set_tacho_command_inx(motorsIndex, TACHO_RESET);
    Sleep(100);
    set_tacho_stop_action_inx( motorsIndex[0], TACHO_COAST);
    set_tacho_stop_action_inx( motorsIndex[1], TACHO_COAST);
    set_tacho_stop_action_inx( motorsIndex[2], TACHO_COAST);
    Sleep(100);
}

// MOVEMENTS
void goStraigth(uint8_t *m, INX_T command, float speed, int time)
{
    //Motor 0
    set_tacho_speed_sp( m[0], speed);
    set_tacho_ramp_up_sp( m[0], 100);
    set_tacho_ramp_down_sp( m[0], 100);
    set_tacho_time_sp( m[0], time);
    //Motor 1
    set_tacho_speed_sp( m[1], speed);
    set_tacho_ramp_up_sp( m[1], 100);
    set_tacho_ramp_down_sp( m[1], 100);
    set_tacho_time_sp( m[1], time);
    doubleSetCommand(m[0], m[1], command); // We should dedice how much go straight and in which way
    if ( command != TACHO_RUN_FOREVER ) {
    waitMotorEnds(m[0]);
    waitMotorEnds(m[1]);}
    }

void turn(uint8_t *m, uint8_t *s, float degreeRight, float speedRight, int gap, float *ret)
{
    float stopAng,sonarCurrent,angleCurrent,prec;
    int lr=1;
    objects obj1;

    if (degreeRight < 0) lr = -1;
    if (gap){
    obj1.firstDistance = 5000;
    obj1.lastDistance = 5000;
    obj1.minDistance = 5000;
    obj1.firstAngle = 0;
    obj1.lastAngle = 0;
    obj1.detected = false;}
    //Motor B
    set_tacho_speed_sp( m[0], -speedRight );
    set_tacho_ramp_up_sp( m[0], 100 );
    set_tacho_ramp_down_sp( m[0], 100 );
    //Motor C
    set_tacho_speed_sp( m[1], speedRight );
    set_tacho_ramp_up_sp( m[1], 100 );
    set_tacho_ramp_down_sp( m[1], 100 );

    angleCurrent = getGyroValue(s[3]);
    stopAng = lr * (angleCurrent + degreeRight);
    prec = getSonarValue(s[2]);

    doubleSetCommand(m[0], m[1], TACHO_RUN_FOREVER);
    while(stopAng >= lr * angleCurrent)//while turning not ended; if turnLeft the lr correct the expression
    {
        angleCurrent = getGyroValue(s[3]);

        if(gap)//if search ball
        {
            sonarCurrent = getSonarValue(s[2]);
            if((prec-sonarCurrent) > gap)//opening obj detected only for 20cm gap!
            {
                if(sonarCurrent < obj1.firstDistance)//is this obj nearer?
                {
                    obj1.firstDistance = sonarCurrent;
                    obj1.minDistance = sonarCurrent;
                    obj1.firstAngle = angleCurrent;
                    obj1.detected = true;
                }
            }
            if(obj1.detected && obj1.minDistance > sonarCurrent) obj1.minDistance = sonarCurrent;
            if((sonarCurrent-prec) > gap)//closing obj
            {
                if(prec < obj1.lastDistance)
                {
                    obj1.lastDistance = prec;
                    obj1.lastAngle = angleCurrent;
                    obj1.detected = false;
                }
            }
            prec = sonarCurrent;
        }
    }
    doubleSetCommand(m[0], m[1], TACHO_STOP);
    ret[0] = obj1.minDistance;
    ret[1] = (obj1.firstAngle + obj1.lastAngle)/2;
    Sleep(200);
   // printf("min sonar %f min angle %f \n",ret[0],ret[1]);
}

void waitMotorEnds(uint8_t sn)
{
    int flag;
    FLAGS_T state;

    do
    {
        get_tacho_state_flags( sn, &state ); //control motor's status
        flag = state & TACHO_RUNNING;//and between status bits
        if(flag == TACHO_RUNNING)//verify if motor is running
            flag = (state == 25) ? 0 : 1;//?probably another bit is set if overloaded?
    }
    while (flag == 1);
}

void handsCalibration(uint8_t hAdd)
{
    set_tacho_position(hAdd, -4);//correct the predefined position
}

bool catchBall(uint8_t *motorId, uint8_t *s, int timeDistance)
{
    int pos[3] = {0, -120, -230}, velStraigth = 200, velHands = 200, col[2]={0,1};//abs positions
    bool ballFound = false;

    set_tacho_speed_sp(motorId[2], velHands);
    set_tacho_position_sp (motorId[2], pos[2]);
    singleSetCommand(motorId[2], TACHO_RUN_TO_ABS_POS);
    waitMotorEnds(motorId[2]);
    goStraigth(motorId, TACHO_RUN_TIMED, velStraigth, timeDistance);

    set_tacho_position_sp (motorId[2], pos[0]);
    singleSetCommand(motorId[2], TACHO_RUN_TO_ABS_POS);
    waitMotorEnds(motorId[2]);
    Sleep(1000);
        while(col[1] != col[2]){
    col[1] = getColorValue(s[1]);
    Sleep(500);
    col[2] = getColorValue(s[1]);
    }
    if(col[1] == 2 || col[1] == 5){
        ballFound = true;
        goStraigth(motorId, TACHO_RUN_TIMED, -velStraigth, timeDistance);
    }
    else{
        leaveBall(motorId, timeDistance);
    }
    return ballFound;
}

void leaveBall(uint8_t *motorId, int timeDistance)
{
    int pos[3] = {0, -120, -230}, velHands = 150, vel = 400;//abs positions

    set_tacho_speed_sp(motorId[2], velHands);//speed to be set when initialized
    set_tacho_position_sp (motorId[2], pos[1]);
    singleSetCommand(motorId[2], TACHO_RUN_TO_ABS_POS);
    waitMotorEnds(motorId[2]);
    Sleep(1000);
    set_tacho_position_sp (motorId[2], pos[2]);
    singleSetCommand(motorId[2], TACHO_RUN_TO_ABS_POS);
    waitMotorEnds(motorId[2]);

    goStraigth(motorId, TACHO_RUN_TIMED, -vel, timeDistance);

    set_tacho_position_sp (motorId[2], pos[0]);
    singleSetCommand(motorId[2], TACHO_RUN_TO_ABS_POS);
    waitMotorEnds(motorId[2]);
}
