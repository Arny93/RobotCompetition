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
#endif
#define CORR_R -2
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

//CORRECTION APPLIED WHEN THE ROBOT ARRIVE AT THE BASE
void arrivalCorrection(uint8_t *m, uint8_t *s, bool turnRight){
    float tmp, ret[2], mult = 3.5;
    int distance = 200, turnVel= 60, strVel=400, crrR = CORR_R*2, crrL = CORR_L*2, span=20;
    crrR = 90 + crrR;
    crrL = crrL - 90;

    searchBall(m,s,span,1,ret);

    tmp = ret[0];
    if(tmp < distance){
        tmp = distance - tmp;
        goStraigth(m, TACHO_RUN_TIMED, -strVel, tmp*mult);
    }
    else{
        tmp = tmp - distance;
        goStraigth(m, TACHO_RUN_TIMED, strVel, tmp*mult);
    }

    if(turnRight){
        turn(m,s,crrR,turnVel,0,ret);
    }
    else{
        turn(m,s,crrL,-turnVel,0,ret);
    }

    searchBall(m,s,span,1,ret);

    tmp = ret[0];
    if(tmp < distance){
        tmp = distance - tmp;
        goStraigth(m, TACHO_RUN_TIMED, -strVel, tmp*mult);
    }
    else{
        tmp = tmp - distance;
        goStraigth(m, TACHO_RUN_TIMED, strVel, tmp*mult);
    }

    if(turnRight){
        turn(m,s,crrR,turnVel,0,ret);
    }
    else{
        turn(m,s,crrL,-turnVel,0,ret);
    }
}

void leaveLittleStadium(uint8_t *m,  uint8_t *s, int beginner){
    float ret[2], app;
    int test, turnVel=60, strVel=400, crrR = CORR_R*2, crrL = CORR_L*2;
    crrR = crrR + 90;
    crrL = crrL - 90;

    //BEGINNER
    if(beginner == 0){
        handsCalibration(m[2]);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 3400);
        turn(m, s, crrL, -turnVel, 0, ret);
        goStraigth(m, TACHO_RUN_TIMED, strVel, 1150);

        leaveBall(m,1150);
        turn(m, s, crrR, turnVel,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 3600);
        arrivalCorrection(m,s,true);
    }
    else if(beginner == 1){
        goStraigth(m, TACHO_RUN_TIMED, strVel, 3400);
        turn(m, s, crrR, turnVel,0, ret);
        goStraigth(m,TACHO_RUN_TIMED,strVel,300);
        test = getGyroValue(s[3]);
        searchBall(m,s,50,200,ret);
        while (!catchBall(m,s, ret[0]*7.1)){
            searchBall(m,s,100,100,ret);
        }
        app = test - ret[1];
        if(app<0){
            turn(m, s, app , -turnVel,0, ret);
        }
        else{
            turn(m, s, app , turnVel,0, ret);
        }
        turn(m, s, crrL, -turnVel,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 3600);
        arrivalCorrection(m,s,false);
    }
    //FINISHER
    else if(beginner == 2){
        goStraigth(m, TACHO_RUN_TIMED, strVel, 3400);
        turn(m, s, crrL, -turnVel, 0, ret);
        test = getGyroValue(s[3]);
        searchBall(m,s,50,200,ret);
        while (!catchBall(m,s, ret[0]*7.1)){
            searchBall(m,s,100,100,ret);
        }
        app = test - ret[1];

        turn(m, s, crrR + app , turnVel,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 3600);
        arrivalCorrection(m,s,true);
    }
    else{
        goStraigth(m, TACHO_RUN_TIMED, strVel, 3400);
        turn(m, s, crrR, turnVel,0, ret);
        test = getGyroValue(s[3]);
        goStraigth(m, TACHO_RUN_TIMED, strVel, 1150);
        leaveBall(m,1150);
        turn(m, s, crrL, -turnVel,0, ret);

        goStraigth(m, TACHO_RUN_TIMED, strVel, 3600);
        arrivalCorrection(m,s,false);
    }
}
