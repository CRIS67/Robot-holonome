/*
 * File:   main.c
 * Author: Quentin BOYER
 *
 * Created on September 18, 2017, 6:59 PM
 */

#include <xc.h>
#include <math.h>
#include <p33EP512GM310.h>
#include <stdint.h>
#include "constant.h"
#include "clock.h"
#include "GPIO.h"
#include "timer.h"
#include "PWM.h"
#include "QEI.h"
#include "PID.h"
#include "SPI.h"
#include "ADC.h"
#include "UART.h"
#include "interrupt.h"
#include "US.h"
#include "AX12.h"
#include "DMA.h"

void go(double arg_cx, double arg_cy, double arg_ct, double arg_speedMax, double accMax);

// <editor-fold defaultstate="collapsed" desc="Variables">

//Global variables
//char TX[TX_SIZE];
char RX[RX_SIZE];
char unsigned TX_i;
char unsigned RX_i;
/*Current position*/
volatile long double x;
volatile long double y;
volatile long double theta;
/*Current setpoint position*/
volatile long double xc;
volatile long double yc;
volatile long double thetac;
/*Final setpoint position*/
volatile long double xf;
volatile long double yf;
volatile long double tf;

uint8_t finalPoint = 1;

volatile PID pidSpeedLeft, pidSpeedRight, pidDistance, pidAngle;
volatile PID pidSpeed0, pidSpeed1, pidSpeed2;
int state = 0;
int R, L;

//volatile char arrived;
volatile char arrived_2;

extern volatile char US_ON[NB_US];
extern volatile char US_R[NB_US];
extern volatile double US[NB_US];

volatile char sendBT = 0;

extern volatile double receivedX, receivedY, receivedTheta;
extern volatile unsigned char newPosReceived;
extern volatile unsigned char newPosBackReceived;
extern volatile unsigned char newAngleReceived;
extern volatile unsigned char back;

extern volatile unsigned char debugPosRpi;

volatile unsigned char stop = 0;

unsigned char detectUS = 0;
unsigned char sensDetectUS = 1;

extern double var_ACC_MAX;

// <editor-fold defaultstate="collapsed" desc="Génération de trajectoire">
extern unsigned char statePathGeneration;
extern unsigned char stateTrap;

extern double cx, cy, ct;

extern double sign;

extern double phi;
extern double angle;

extern double theta0;
extern double angularVelocity;
extern double prevAngularVelocity;
extern double maxAngularVelocity;

extern double AngularAcceleration;

extern double angle1;


extern double dx;
extern double dy;
extern double alpha;

extern double totalDistance;
extern double dist;

extern double y_0;
extern double x_0;
extern double speed;
extern double precSpeed;

extern double acc;
extern double speedMax;

extern double dist1; // </editor-fold>

extern uint8_t BufferA[8];
extern volatile uint8_t RxDMABuffer[RX_DMA_SIZE];
extern uint16_t start;

extern uint16_t iD2, iF2;

volatile uint8_t verbose = 0;

volatile int16_t tick0 = 0;
volatile int16_t tick1 = 0;
volatile int16_t tick2 = 0;

volatile double funSpeed = 1000;
volatile double funAcc = 1000;

volatile double funAngularSpeed = 10;
volatile double funAngularAcc = 1;

volatile long double linSpeed = 0; //mm/s
volatile long double linAcc = 0; //mm/s^2
volatile long double rotSpeed = 0; //rad/s
volatile long double rotAcc = 0; //rad/s^2

int sens = 0;


volatile long double kahanErrorX;
volatile long double kahanErrorY;
volatile long double kahanErrorT;

volatile uint8_t arrived = 1;

extern volatile long double alphaDebug;

extern volatile long double commandeXDebug;
extern volatile long double commandeYDebug; // </editor-fold>


extern long double receivedSpeedX;
extern long double receivedSpeedY;
extern long double receivedSpeedT;

int main(){
    initClock(); //Clock 140 MHz
    initGPIO();
    initPWM();
    initDMA();
    initUART();
    initInt();
    
    initAllPID(&pidSpeed0, &pidSpeed1, &pidSpeed2, &pidDistance, &pidAngle);

    initTimer();
    initADC();
    //initSPI();
    
    
    initPWM();
    x = 1000;
    y = 1500;
    xc = x;
    yc = y;
    xf = x;
    yf = y;
    theta = 0;
    
    stop = 1;
    pidAngle.setPoint = thetac;
    while(1){
        CheckMessages();
        sendPos();
        if(newPosReceived){
                /*xf = receivedX;
                yf = receivedY;
                arrived = 0;
                statePathGeneration = 42;
                delay_ms(100);*/
                newPosReceived = 0;
                
                go(receivedX,receivedY,0,linSpeed,linAcc);
        }
        delay_ms(10);
        plot(1,(uint32_t)(int32_t)(pidSpeed0.setPoint*1000));
        plot(2,(uint32_t)(int32_t)(pidSpeed1.setPoint*1000));
        plot(3,(uint32_t)(int32_t)(pidSpeed2.setPoint*1000));
        plot(4,(uint32_t)(int32_t)(pidSpeed0.processVariable*1000));
        plot(5,(uint32_t)(int32_t)(pidSpeed1.processVariable*1000));
        plot(6,(uint32_t)(int32_t)(pidSpeed2.processVariable*1000));
        
        plot(11,(uint32_t)(int32_t)receivedSpeedX);
        //plot(12,(uint32_t)(int32_t)x);
        
        plot(21,(uint32_t)(int32_t)receivedSpeedY);
        //plot(22,(uint32_t)(int32_t)y);
        
        //plot(31,(uint32_t)(int32_t)thetac);
        plot(32,(uint32_t)(int32_t)(receivedSpeedT*1800/PI));
    }
    return 0;
}


void go(double arg_cx, double arg_cy, double arg_ct, double arg_speedMax, double accMax){
    /*INIT*/
    
    cx = arg_cx;
    cy = arg_cy;
    ct = arg_ct;
    
    speedMax = arg_speedMax;
    
    double thetaRobotPoint = atan2(cy-y,cx-x);
    phi = thetaRobotPoint - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    finalPoint = 0;
    theta0 = theta;
    AngularAcceleration = funAngularAcc;
    maxAngularVelocity = funAngularSpeed;
    /*AngularAcceleration = 1;
    maxAngularVelocity = 10;*/
    angularVelocity = 0;
    prevAngularVelocity = 0;
    angle = 0;
    
    if(phi > 0)
        sign = 1;
    else{
        sign = -1;
        phi = -phi;
    }
    
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    acc = accMax;
	speed = 0;
	precSpeed = 0;
	dist = 0;
    xf = arg_cx;
    yf = arg_cy;
    statePathGeneration = 1;
    stateTrap = 1;
}
