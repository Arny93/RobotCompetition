#include "fmotor.h"
#include "fsensor.h"
#include "ev3_tacho.h"
#include "actions.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#ifndef _sleepFunction
#define _sleepFunction
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif // _sleepFunction
#define CORR_R -1
#define CORR_L 2

void searchBall(uint8_t *m,  uint8_t *s, int span, int gap, float *res)
{
    float ret[2], compare[4], medium[2] = {0,0};
    int val, turnVel=60, crrR = CORR_R, crrL = CORR_L;
    bool finished = false;

    turn(m, s, crrL-span, -turnVel, 0, ret);//turnLeft
    do{
    turn(m, s, (span + crrR)*2, turnVel, gap, ret);//turnRight
    compare[0] = ret[0];
    compare[1] = ret[1];
    medium[0] += ret[0];
    medium[1] += ret[1];
    turn(m,s, (crrL-span)*2, -turnVel, gap, ret);
    compare[2] = ret[0];
    compare[3] = ret[1];
    medium[0] += ret[0];
    medium[1] += ret[1];
    medium[0] = medium[0] / 2;
    medium[1] = medium[1] / 2;
    if (medium[1] - compare[1] < 10 && medium[1] - compare[3] < 10)
        finished = true;
    } while (!finished);

    res[0] = medium[0];
    res[1] = medium[1];
    val = medium[1] - getGyroValue(s[3]);
    turn(m, s, val + crrR, turnVel, 0, ret);//lef
}

void leaveBiggerStadium(uint8_t *m,  uint8_t *s, int begSide)
{
    float ret[2],test, turnVelocity = 60,strVel=600,app;
    int crrR = CORR_R*2, crrL = CORR_L*2,  span=20;
    crrR = crrR + 90;
    crrL = crrL - 90;

    if(begSide==0){ //BEGINNER RIGHT TEAM
        turn(m, s, 25+CORR_R*2, turnVelocity, 0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 4900);
        turn(m,s, -25+CORR_L*2, -turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_FOREVER, 600, 2800);
        do
        {
            test=getSonarValue(s[2]);
        }
        while(test > 330);
        doubleSetCommand(m[0], m[1], TACHO_STOP);
        searchBall(m,s,span,1,ret);

        turn(m, s, crrR,turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, -strVel, 1400);
        leaveBall(m, 1500);

        turn(m, s, crrL, -turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 5900);
    }

    else if(begSide==1){ //BEGINNER LEFT TEAM
        turn(m, s, -25+CORR_L*2, -turnVelocity, 0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 5100);
        turn(m,s, 25+CORR_R*2, turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_FOREVER, 600, 2800);
        do
        {
            test=getSonarValue(s[2]);
        }
        while(test > 330);
        doubleSetCommand(m[0], m[1], TACHO_STOP);
        searchBall(m,s,span,1,ret);

        turn(m, s, crrL,-turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, -strVel, 1400);
        leaveBall(m, 1500);

        turn(m, s, crrR, turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 5900);
    }

    else if(begSide==2){ //FINISHER RIGHT TEAM
        turn(m, s, 25+CORR_R*2, turnVelocity, 0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 4900);
        turn(m,s, -25+CORR_L*2, -turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_FOREVER, 600, 2800);
        do
        {
            test=getSonarValue(s[2]);
        }
        while(test > 330);
        doubleSetCommand(m[0], m[1], TACHO_STOP);
        searchBall(m,s,span,1,ret);

        turn(m, s, crrL,-turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 500);

        test = getGyroValue(s[3]);
        searchBall(m,s,50,200,ret);
    while (!catchBall(m,s, ret[0]*7.1)){
    searchBall(m,s,100,100,ret);
    }
        app = test - ret[1];
        if(abs(app)>5){
        if(app>0){
            turn(m, s, app+CORR_R*2, turnVelocity, 0, ret);
        }
        else{
            turn(m, s, app+CORR_L*2,-turnVelocity,0, ret);
        }
        goStraigth(m, TACHO_RUN_TIMED, strVel, 2300);
        turn(m, s, crrR, turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 5900);
        }
    }

    else{ //FINISHER LEFT TEAM
        turn(m, s, -25+CORR_L*2, -turnVelocity, 0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 5100);
        turn(m,s, 25+CORR_R*2, turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_FOREVER, 600, 2800);
        do
        {
            test=getSonarValue(s[2]);
        }
        while(test > 330);
        doubleSetCommand(m[0], m[1], TACHO_STOP);
        searchBall(m,s,span,1,ret);

        turn(m, s, crrR, turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 500);

        test = getGyroValue(s[3]);
        searchBall(m,s,50,200,ret);
    while (!catchBall(m,s, ret[0]*7.1)){
    searchBall(m,s,100,100,ret);
    }        app = test - ret[1];

        if(app>0){
            turn(m, s, app+CORR_R*2, turnVelocity, 0, ret);
        }
        else{
            turn(m, s, app+CORR_L*2,-turnVelocity,0, ret);
        }

        goStraigth(m, TACHO_RUN_TIMED, strVel, 2300);
        turn(m, s, crrL, -turnVelocity,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 5900);
    }
}
