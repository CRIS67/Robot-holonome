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



void testAccMax();
void reglageDiametre();
void straightPath(double cx, double cy, double ct, double speedMax, double accMax);
void go(double cx, double cy, double speedMax, double accMax);
void goBack(double cx, double cy, double speedMax, double accMax);
void turn(double ct);
void turnNOLIMIT(double ct);

void modif_straightPath(double arg_cx, double arg_cy, double arg_ct, double arg_speedMax, double accMax);

void test();
void test2();
void testPWM();
void testSPI();
void testADC();
void testInterrupt();
void testDelay();

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
int R,L;

volatile char arrived;
volatile char arrived_2;

extern volatile char   US_ON[NB_US];
extern volatile char   US_R[NB_US];
extern volatile double US[NB_US];

volatile char sendBT = 0;

extern volatile double receivedX,receivedY,receivedTheta;
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

extern uint16_t iD2,iF2;

volatile uint8_t verbose = 0;

volatile int16_t tick0 = 0;
volatile int16_t tick1 = 0;
volatile int16_t tick2 = 0;

volatile double funSpeed = 1000;
volatile double funAcc = 1000;

volatile double funAngularSpeed = 10;
volatile double funAngularAcc = 1;

int sens = 0;


volatile long double kahanErrorX;
volatile long double kahanErrorY;
volatile long double kahanErrorT;


int main(){
    initClock(); //Clock 140 MHz
    initGPIO();
    initPWM();
    //initQEI();
    initDMA();
    initUART();
    //initAX12();
    initInt();
    
    initPID(&pidSpeed0,KP_SPEED_0,KI_SPEED_0,KD_SPEED_0,BIAS_SPEED_0,0,T_SPEED_0,0,0,SMOOTHING_FACTOR_SPEED_0,SATURATION_SPEED_0);
    initPID(&pidSpeed1,KP_SPEED_1,KI_SPEED_1,KD_SPEED_1,BIAS_SPEED_1,0,T_SPEED_1,0,0,SMOOTHING_FACTOR_SPEED_1,SATURATION_SPEED_1);
    initPID(&pidSpeed2,KP_SPEED_2,KI_SPEED_2,KD_SPEED_2,BIAS_SPEED_2,0,T_SPEED_2,0,0,SMOOTHING_FACTOR_SPEED_2,SATURATION_SPEED_2);

    initTimer();
    //initUS();
    initADC();
    //initSPI();
    
    
    /*
    LATBbits.LATB10 = 1;    //PWM_ASS_0
    LATGbits.LATG0 = 1;     //SENS_ASS_0
    
    LATBbits.LATB11 = 1;    //PWM_ASS_1
    LATGbits.LATG1 = 1;     //SENS_ASS_1
    
    LATBbits.LATB13 = 1;    //PWM_ASS_2
    LATGbits.LATG12 = 1;     //SENS_ASS_2
    */
    
    
    //uint16_t w = 0;
    //uint16_t w2 = 0;
    
    initPWM();
    int iLed = 0;
    while(!verbose){
        CheckMessages();
    }
    /*while(1){
        plot(1,(uint32_t)((int32_t)(tick0)));
        delay_ms(10);
        CheckMessages();
    }*/
    double iMotor = 0;
    int16_t saveTick0 = tick0;
    int16_t saveTick1 = tick1;
    int16_t saveTick2 = tick2;
    /*while(1){
        delay_ms(1000);
        sendLog("hello\n");
    }*/
    IEC0bits.T1IE = 1;
    while(1){
        setSetPoint(&pidSpeed0,1);
        setSetPoint(&pidSpeed1,-1);
        setSetPoint(&pidSpeed2,0);
        for(iMotor = 0; iMotor < 200; iMotor++){
            CheckMessages();
            delay_ms(10);
        }
        setSetPoint(&pidSpeed0,-1);
        setSetPoint(&pidSpeed1,1);
        setSetPoint(&pidSpeed2,0);
        for(iMotor = 0; iMotor < 200; iMotor++){
            CheckMessages();
            delay_ms(10);
        }
    }
    while(1){
        for(iMotor = 10; iMotor < 50; iMotor+=0.02){
            delay_ms(20);
            int32_t s0 = tick0 - saveTick0;
            int32_t s1 = tick1 - saveTick1;
            int32_t s2 = tick2 - saveTick2;
            saveTick0 = tick0;
            saveTick1 = tick1;
            saveTick2 = tick2;
            motor(0,iMotor);
            motor(1,iMotor);
            motor(2,iMotor);
            plot(1,(uint32_t)((int32_t)(iMotor*100)));
            plot(2,(uint32_t)((int32_t)(s0)));
            plot(3,(uint32_t)((int32_t)(s1)));
            plot(4,(uint32_t)((int32_t)(s2)));
            plot(5,(uint32_t)((int32_t)(PWM_0)));
        }
        sendMotor(0,0,0);
        delay_ms(2000);
        for(iMotor = 0; iMotor >-50; iMotor--){
            delay_ms(100);
            int32_t s0 = tick0 - saveTick0;
            int32_t s1 = tick1 - saveTick1;
            int32_t s2 = tick2 - saveTick2;
            saveTick0 = tick0;
            saveTick1 = tick1;
            saveTick2 = tick2;
            motor(0,iMotor);
            motor(1,iMotor);
            motor(2,iMotor);
            plot(1,(uint32_t)((int32_t)(iMotor*10)));
            plot(2,(uint32_t)((int32_t)(s0)));
            plot(3,(uint32_t)((int32_t)(s1)));
            plot(4,(uint32_t)((int32_t)(s2)));
        }
        sendMotor(0,0,0);
        delay_ms(2000);
    }
    /*while(1){
        plot(1,(uint32_t)((int32_t)(millis())));
        delay_ms(50);
    }*/
    /*for(iMotor = 0; iMotor < 100; iMotor++){
            delay_ms(20);
            int32_t s0 = tick0 - saveTick0;
            int32_t s1 = tick1 - saveTick1;
            int32_t s2 = tick2 - saveTick2;
            saveTick0 = tick0;
            saveTick1 = tick1;
            saveTick2 = tick2;
            motor(0,iMotor);
            motor(1,iMotor);
            motor(2,iMotor);
            plot(1,(uint32_t)((int32_t)(iMotor)));
            plot(2,(uint32_t)((int32_t)(s0)));
            plot(3,(uint32_t)((int32_t)(s1)));
            plot(4,(uint32_t)((int32_t)(s2)));
        }
        for(iMotor = 100; iMotor > -100; iMotor--){
            delay_ms(20);
            int32_t s0 = tick0 - saveTick0;
            int32_t s1 = tick1 - saveTick1;
            int32_t s2 = tick2 - saveTick2;
            saveTick0 = tick0;
            saveTick1 = tick1;
            saveTick2 = tick2;
            motor(0,iMotor);
            motor(1,iMotor);
            motor(2,iMotor);
            plot(1,(uint32_t)((int32_t)(iMotor)));
            plot(2,(uint32_t)((int32_t)(s0)));
            plot(3,(uint32_t)((int32_t)(s1)));
            plot(4,(uint32_t)((int32_t)(s2)));
        }
        for(iMotor = -100; iMotor < 0; iMotor++){
            delay_ms(20);
            int32_t s0 = tick0 - saveTick0;
            int32_t s1 = tick1 - saveTick1;
            int32_t s2 = tick2 - saveTick2;
            saveTick0 = tick0;
            saveTick1 = tick1;
            saveTick2 = tick2;
            motor(0,iMotor);
            motor(1,iMotor);
            motor(2,iMotor);
            plot(1,(uint32_t)((int32_t)(iMotor)));
            plot(2,(uint32_t)((int32_t)(s0)));
            plot(3,(uint32_t)((int32_t)(s1)));
            plot(4,(uint32_t)((int32_t)(s2)));
        }
    sendMotor(0,0,0);*/
    
    while(1){   /*Chenillard*/
        LATFbits.LATF5  = 0;
        LATGbits.LATG3  = 1;
        delay_ms(100);
        LATGbits.LATG3  = 0;
        LATFbits.LATF4  = 1;
        delay_ms(100);
        LATFbits.LATF4  = 0;
        LATFbits.LATF5  = 1;
        delay_ms(100);
        CheckMessages();
        //sendLog("hello there =) !\n");
    }
    return 0;
}

void testAccMax(){
    double commandeL = 12;
    double prevCommandeL = 0;
    double delta = 0.1;
    double newCL;
    while(1){
        if(commandeL - prevCommandeL > delta)
            newCL = prevCommandeL + delta;
        else if(commandeL - prevCommandeL < -delta)
            newCL = prevCommandeL - delta;
        else
            newCL = commandeL;
        prevCommandeL = newCL;
        //sendToMotor(0,newCL);
        delay_ms(30);
    }
}
void reglageDiametre(){
    IEC0bits.T1IE = 0;
    POS1CNTL = 0x8000;
    POS2CNTL = 0x8000;
    double l,r,l2,r2;
    double D1,D2;
    while(1){
        l = (double)(POS1CNTL - 0x8000) * PI * 2 * ENCODER_WHEEL_RADIUS / 4096;
        r = (POS2CNTL - 0x8000) * PI * 2 * ENCODER_WHEEL_RADIUS / 4096;
        l2 = (double)(POS1CNTL - 0x8000) * PI * 2 / 4096;
        r2 = (double)(POS2CNTL - 0x8000) * PI * 2 / 4096;
        D1 = 300/l2;
        D2 = 300/r2;
        
        /*print("l : ");
        print(itoa((int)l));
        print("      r : ");
        print(itoa((int)r));
        print("      D1 : ");
        print(itoa((int)(D1)));
        print("      D1*100 : ");
        print(itoa((int)(D1*100)));
        print("      D2 : ");
        print(itoa((int)(D2)));
        print("      D2*100 : ");
        print(itoa((int)(D2*100)));
       
        print("\r\n");*/
        delay_ms(200);
    }   
}
void straightPath(double cx, double cy, double ct, double speedMax, double accMax){
    double i;
    double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = thetaRobotPoint - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 10;
    double maxAngularVelocity = 10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0)
        sign = 1;
    else
        sign = -1;
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    delay_ms(500);
    /* Phase 2 : straight line */
    xf = cx;
    yf = cy;
    tf = ct;
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    double y0 = y;
	double x0 = x;
	double dx = cx - x0;
	double dy = cy - y0;
	double alpha = atan2(dy,dx);
	double totalDistance = sqrt(dx*dx+dy*dy);
    
    double acc = accMax;
	double speed = 0;
	double precSpeed = 0;
	double dist = 0;
	while(speed < speedMax && dist < totalDistance/2){
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    double dist1 = dist;
	//2
	if(speed > speedMax)
        speed = speedMax;
	while(dist < totalDistance - dist1){               //Condition
		dist += TE * speed;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
	//3
	acc = -acc;
	while(speed > 0 && dist < totalDistance){				//Condition		//v > 0			/		d < totalDistance
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    xc = cx;
    yc = cy;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    delay_ms(500);
    //while(!arrived);
    
    /*Phase 3 : rotation */
    if(ct > theta){
        for(i = theta; i <= ct; i+= ROTATION_SPEED){
            thetac = i;
            delay_ms(DELAY_SPEED);
        }
    }
    else{
        for(i = theta; i >= ct; i-= ROTATION_SPEED){
            thetac = i;
            delay_ms(DELAY_SPEED);
        }
    }
    //print("Phase 3 OK\n");
    thetac = ct;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    delay_ms(500);
    //while(!arrived);
}

void go(double cx, double cy, double speedMax, double accMax){
    arrived_2 = 0;
    double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = thetaRobotPoint - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 3;//1;//10; envoie du steak
    double maxAngularVelocity = 3;//1;//10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0){
        sign = 1;
    }
    else{
        sign = -1;
        phi = -phi;
    }
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    //delay_ms(100);
    /* Phase 2 : straight line */
    xf = cx;
    yf = cy;
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    double y0 = y;
	double x0 = x;
	double dx = cx - x0;
	double dy = cy - y0;
	double alpha = atan2(dy,dx);
	double totalDistance = sqrt(dx*dx+dy*dy);
    
    double acc = accMax;
	double speed = 0;
	double precSpeed = 0;
	double dist = 0;
	while(speed < speedMax && dist < totalDistance/2){
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    double dist1 = dist;
	//2
	if(speed > speedMax)
        speed = speedMax;
	while(dist < totalDistance - dist1){               //Condition
		dist += TE * speed;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
	//3
	acc = -acc;
	while(speed > 0 && dist < totalDistance){				//Condition		//v > 0			/		d < totalDistance
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    xc = cx;
    yc = cy;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    //delay_ms(100);
    //while(!arrived);
}

void goBack(double cx, double cy, double speedMax, double accMax){
    arrived_2 = 0;
    //printRpi("DEBUG begin of goBack()");
    back = 1;
    double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = thetaRobotPoint - theta - PI;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 3;//10; envoie du steak
    double maxAngularVelocity = 3;//10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0){
        sign = 1;
    }
    else{
        sign = -1;
        phi = -phi;
    }
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    //delay_ms(500);
    /* Phase 2 : straight line */
    xf = cx;
    yf = cy;
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    double y0 = y;
	double x0 = x;
	double dx = cx - x0;
	double dy = cy - y0;
	double alpha = atan2(dy,dx);
	double totalDistance = sqrt(dx*dx+dy*dy);
    
    double acc = accMax;
	double speed = 0;
	double precSpeed = 0;
	double dist = 0;
	while(speed < speedMax && dist < totalDistance/2){
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    double dist1 = dist;
	//2
	if(speed > speedMax)
        speed = speedMax;
	while(dist < totalDistance - dist1){               //Condition
		dist += TE * speed;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
	//3
	acc = -acc;
	while(speed > 0 && dist < totalDistance){				//Condition		//v > 0			/		d < totalDistance
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    xc = cx;
    yc = cy;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    //delay_ms(500);
    back = 0;
    //printRpi("DEBUG end of goBack()\n");
    //while(!arrived);
}

void turn(double ct){
    //double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = ct - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 3;//1;//10; envoie du steak
    double maxAngularVelocity = 3;//1;//10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0){
        sign = 1;
    }
    else{
        sign = -1;
        phi = -phi;
    }
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    delay_ms(500);
    
}
void modif_straightPath(double arg_cx, double arg_cy, double arg_ct, double arg_speedMax, double accMax){
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

    statePathGeneration = 1;
    stateTrap = 1;
}
void turnNOLIMIT(double ct){
    //double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = ct - theta;
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 3;//1;//10; envoie du steak
    double maxAngularVelocity = 10;//1;//10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0){
        sign = 1;
    }
    else{
        sign = -1;
        phi = -phi;
    }
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    delay_ms(500);
    
}
void testPIDs(){
    while(1){
        
        x = 0;
        y = 0;
        theta = 0;
        xc = x;
        yc = y;
        thetac = theta;
        xf = x;
        yf = y;
        tf = theta;
        
        while(stop);
        
        modif_straightPath(300,300,0,1000,100);
        
    }
}