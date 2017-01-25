#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "fmotor.h"
#include "fsensor.h"
#include "actions.h"
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <stdarg.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#define SERV_ADDR   "00:26:B6:7F:A6:F7"     /* Whatever the address of the server is */
#define TEAM_ID     7                       /* Your team ID */

#ifndef _sleepFunction
#define _sleepFunction
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif // _sleepFunction

#define MSG_ACK     0
#define MSG_NEXT    1
#define MSG_START   2
#define MSG_STOP    3
#define MSG_CUSTOM  4
#define MSG_KICK    5
#define MSG_POSITION 6
#define MSG_BALL 	7

int s;
int fin=0;
unsigned char whoAmI = 10;
unsigned char next = 0xFF;
int sendNext = 1;
int receiveNext = 0;
int finitoFin = 0;
uint16_t msgId = 0;

//POSITION
float pos_X = 0;
float pos_Y = 0;

uint8_t motors[3], sensors[4];

int read_from_server (int sock, char *buffer, size_t maxSize) {
    int bytes_read = read (sock, buffer, maxSize);

    if (bytes_read <= 0) {
        fprintf (stderr, "Server unexpectedly closed connection...\n");
        close (s);
        exit (EXIT_FAILURE);
    }

    printf ("[DEBUG] received %d bytes\n", bytes_read);

    return bytes_read;
}

void *positioning(void * args)
{
	float normWheel = M_PI * 5.5 / 360;
	float gyro_ang;
	float noRot,delta;
 	short flag = 0;
	float prevAng  = 0;
	float piVal = 0;
	float oldMotSx = 0;
	float oldMotDx = 0;
	int newMotSx,newMotDx;
	float disp_sx,disp_dx;

	if(whoAmI!=0){
        piVal = -M_PI/2;// M_PI/2 for finisher
	}
	else{
        piVal = M_PI/2;// M_PI/2 for beginner
	}

    while(1){ // SHOULD ALWAYS UPDATE POSITION
        gyro_ang = getGyroValue(sensors[3]);

        noRot = -( gyro_ang - prevAng);		//rotation

        noRot = noRot*M_PI/180;				//rotation to rad
        prevAng = gyro_ang;				//refresh last angle
        get_tacho_position(motors[1],&newMotSx);
        get_tacho_position(motors[0],&newMotDx);
        gyro_ang = gyro_ang*M_PI/180;
        if(flag==1)
            piVal = piVal + noRot;
        else
        {
            piVal = -M_PI/2;
            flag = 1;
        }
        disp_sx = newMotSx - oldMotSx;
        disp_dx = newMotDx - oldMotDx;
        delta = (disp_sx + disp_dx)*normWheel/2;
        oldMotSx = newMotSx;
        oldMotDx = newMotDx;
        pos_X = pos_X + delta * cos( piVal );
        pos_Y = pos_Y + delta * sin( piVal );
    }
 	pthread_exit(NULL);
}

void *thBluetooth(void *arg){
    struct sockaddr_rc addr = { 0 };
    int status,posLeave=0,valLeave;
    pthread_t coordin;

    time_t now;
    int waiting=0;
    struct tm *tm;

    /* allocate a socket */
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    /* set the connection parameters (who to connect to) */
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba (SERV_ADDR, &addr.rc_bdaddr);

    /* connect to server */
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

    /* if connected */
    if( status == 0 ) {
        char string[58];

        /* Wait for START message */
        read_from_server (s, string, 9);
        if (string[4] == MSG_START) {
            printf ("Received start message!\n");
            whoAmI = (unsigned char) string[5];
            next = (unsigned char) string[7];
        }
        if(whoAmI!=0){ //SET INITIAL POSITION BASED ON BEGINNER/FINISHER
            fin=1;
            pos_X = 25;
            pos_Y = 175;
        }
        else{
            pos_X = 95;
            pos_Y = 25;
        }

        //THREAD THAT CALCULATE POSITION
        pthread_create(&coordin,NULL,positioning,NULL);

        while(1){
            if(whoAmI == 0){ //BEGINNER
                while(sendNext){
                    printf("Wait the beginner finishes, calculate position and see when leave the ball\n");
                    now = time(0);
                    tm=localtime(&now);
                    if(tm->tm_sec % 2 == 0 && waiting ==0){
                        *((uint16_t *) string) = msgId++;
                        string[2] = TEAM_ID;
                        string[3] = 0xFF;
                        string[4] = MSG_POSITION;
                        string[5] = pos_X;          /* x */
                        string[6] = 0x00;
                        string[7] = pos_Y;		/* y */
                        string[8]= 0x00;
                        write(s, string, 9);
                        waiting=1;
                    }
                    else{
                        waiting=0;
                    }
                    if(!posLeave && get_tacho_position_sp(motors[2],&valLeave)<(-50)){
                        //BALL drop message
                        *((uint16_t *) string) = msgId++;
                        string[2] = TEAM_ID;
                        string[3] = next;
                        string[4] = MSG_BALL;
                        string[5] = 0x0;
                        string[6] = pos_X;          /* x */
                        string[7] = 0x00;
                        string[8] = pos_Y;		/* y */
                        string[9]= 0x00;
                        write(s, string, 10);
                        posLeave = 1;
                    }
                }
                posLeave = 0;
                //Next message
                *((uint16_t *) string) = msgId++;
                string[2] = TEAM_ID;
                string[3] = next;
                string[4] = MSG_NEXT;
                write(s, string, 5);
                sendNext = 1;
            }
            else{ //FINISHER
                receiveNext = 0;
                if(finitoFin==0){
                    while(!receiveNext){
                        printf("Wait the other robot finishes waiting the next message\n");
                        read_from_server (s, string, 58);
                        if(string[4]==MSG_NEXT){
                            receiveNext=1;
                            sleep(1);
                        }
                    }
                    finitoFin = 1;
                }
                else{
                    while(finitoFin){
                        if(tm->tm_sec % 2 == 0 && waiting ==0){
                            *((uint16_t *) string) = msgId++;
                            string[2] = TEAM_ID;
                            string[3] = 0xFF;
                            string[4] = MSG_POSITION;
                            string[5] = pos_X;          /* x */
                            string[6] = 0x00;
                            string[7] = pos_Y;		/* y */
                            string[8]= 0x00;
                            write(s, string, 9);
                            waiting=1;
                    }
                    else{
                        waiting=0;
                    }
                    if(!posLeave && get_tacho_position_sp(motors[2],&valLeave)<(-50)){
                            //BALL pick message
                            *((uint16_t *) string) = msgId++;
                            string[2] = TEAM_ID;
                            string[3] = next;
                            string[4] = MSG_BALL;
                            string[5] = 0x1;
                            string[6] = pos_X;          /* x */
                            string[7] = 0x00;
                            string[8] = pos_Y;		/* y */
                            string[9]= 0x00;
                            write(s, string, 10);
                            posLeave = 1;
                        }
                        printf("Wait the finisher finishes and calculate position\n");
                    }
                    whoAmI = 0;
                }
            }
        }

        close (s);

        sleep (5);

    } else {
        fprintf (stderr, "Failed to connect to server...\n");
        sleep (2);
        exit (EXIT_FAILURE);
    }

    close(s);
    pthread_exit(NULL);
}

void *thRobot(void *arg){

    while(whoAmI==10){
        printf("Wait that competitions start\n");
    }

    printf("Competition started\n");

    while(1){
        if(whoAmI == 0){
            printf("The robot is a beginner\n");
            leaveLittleStadium(motors,sensors,0+fin*(3)); //As a beginner
            whoAmI = 2;
            sendNext = 0;
        }
        else{
            while(!receiveNext){
               printf("Wait the other robot\n");
            }

            printf("The other robot has finished\nThe robot is a finisher\n");
            leaveLittleStadium(motors,sensors,1+fin); // As a finisher
            finitoFin = 0;
            receiveNext = 0;
            printf("I finisched the race as a finisher\nI become a beginner\n");
            whoAmI = 0;
        }
    }
    pthread_exit(NULL);
}

int main()
{//address of motors and sensors

    if (ev3_init() == -1 ) return (1);//Initialization of the ev3
    while (ev3_tacho_init() < 1) Sleep(1000);//Initialization of the tacho motors
    searchMotors(motors);//Search the motors
    ev3_sensor_init();//Initialize the sensors
    searchSensors(sensors);//Search the sensors

    pthread_t blue,robot;

    pthread_create(&blue,NULL,thBluetooth,NULL); //Bluetooth thread
    pthread_create(&robot,NULL,thRobot,NULL);    //Robot thread

    pthread_join(robot,NULL);

    return 0;
}
