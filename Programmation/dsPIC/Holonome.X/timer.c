/*
 * File:   timer.c
 * Author: Quentin BOYER
 *
 * Created on 19 septembre 2017, 17:17
 */

#include "timer.h"

// <editor-fold defaultstate="collapsed" desc="Variables">
extern volatile long double x;
extern volatile long double y;
extern volatile long double theta;

extern volatile long double xc;
extern volatile long double yc;
extern volatile long double thetac;

extern volatile long double xf;
extern volatile long double yf;
extern volatile long double tf;

volatile long double kahanX = 0;
volatile long double kahanY = 0;
volatile long double kahanT = 0;

extern volatile long double kahanErrorX;
extern volatile long double kahanErrorY;
extern volatile long double kahanErrorT;

extern uint8_t finalPoint;

extern volatile double receivedX, receivedY, receivedTheta;

extern volatile char US_ON[NB_US];
extern volatile char US_R[NB_US];
extern volatile double US[NB_US];

unsigned int n, n2;

extern volatile PID pidSpeedLeft, pidSpeedRight, pidDistance, pidAngle;
extern volatile PID pidSpeed0, pidSpeed1, pidSpeed2;

double volatile prevCommandeL;
double volatile prevCommandeR;

double volatile prevPrevCommandeL;
double volatile prevPrevCommandeR;


int16_t volatile speedLSum;
int16_t volatile speedRSum;

double volatile smoothSpeedL;
double volatile smoothSpeedR;

//extern volatile char arrived;
extern volatile char arrived_2;

volatile double d;

unsigned char i_us, n_us;

extern volatile char sendBT;
extern volatile unsigned char debugPosRpiAsserv;

extern volatile unsigned char stop;

extern unsigned char detectUS;
extern unsigned char sensDetectUS;
volatile unsigned char hUS = 0;

volatile unsigned char US_received = 0;

volatile unsigned char back = 0;

double var_ACC_MAX = ACC_MAX;

unsigned char timer2Overflow = 0;

long double coef_dissymmetry = COEF_DISSYMETRY;
long double mm_per_ticks = MM_PER_TICKS;
long double rad_per_ticks = RAD_PER_TICKS;
long double distance_between_encoder_wheels = DISTANCE_BETWEEN_ENCODER_WHEELS;

/*Odo holonome*/
volatile long double ticksPerTurn = TICKS_PER_TURN;
volatile long double wheelDiameter = WHEEL_DIAMETER;
volatile long double distanceCenterToWheel = DISTANCE_CENTER_TO_WHEEL;


volatile long double wheelDiameter0 = WHEEL_DIAMETER;
volatile long double wheelDiameter1 = WHEEL_DIAMETER;
volatile long double wheelDiameter2 = WHEEL_DIAMETER;

volatile uint8_t nearPointDistance = 0;
volatile uint8_t nearPointAngle = 0;

volatile uint16_t nplot = 0;

// <editor-fold defaultstate="collapsed" desc="Trajectory generation">
unsigned char statePathGeneration;
unsigned char stateTrap;

double cx, cy, ct;

double sign;

double phi;
double angle;

double theta0;
double angularVelocity;
double prevAngularVelocity;
double maxAngularVelocity;

double AngularAcceleration;

double angle1;


double dx;
double dy;
double alphaTraj;

double totalDistance;
double dist;

double y_0;
double x_0;
double speed;
double precSpeed;

double acc;
double speedMax;

double dist1;
// </editor-fold>

extern volatile int16_t tick0;
extern volatile int16_t tick1;
extern volatile int16_t tick2;

volatile int16_t saveTick0 = 0;
volatile int16_t saveTick1 = 0;
volatile int16_t saveTick2 = 0;

volatile int16_t odoTick0 = 0;
volatile int16_t odoTick1 = 0;
volatile int16_t odoTick2 = 0;

volatile long double s0 = 0;
volatile long double s1 = 0;
volatile long double s2 = 0;

volatile long double prevc0 = 0;
volatile long double prevc1 = 0;
volatile long double prevc2 = 0;

extern volatile uint8_t arrived;

volatile long double alphaDebug = 0;
volatile long double commandeXDebug = 0;
volatile long double commandeYDebug = 0;

volatile long double distanceMax = 10;  //The robot is arrived at its destination if its distance to the destination point is less than this value
volatile long double distanceMax2 = 1;  //The robot is arrived at its destination if its distance to the destination point is less than this value

long double receivedSpeedX = 0;
long double receivedSpeedY = 0;
long double receivedSpeedT = 0;
uint8_t modeAsserv = MODE_POSITION;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Init">
void initTimer() {
    initTimer1(); //Asserv + miscellanous
    initTimer2();   //delay
    //initTimer3();
    //initTimer4();//US
    //initTimer5();//US

    prevCommandeL = 0;
    prevCommandeR = 0;

    speedLSum = 0;
    speedRSum = 0;
    smoothSpeedL = 0;
    smoothSpeedR = 0;

    d = 0;
    i_us = 0;
    n_us = 0;
}
void initTimer1() { //Timer 1 -> asserv' interrupt
    T1CONbits.TON = 0; //disable timer
    TMR1 = 0; // Clear timer register
    PR1 = 1094; //10937;     //period 10 937 -> ~10ms
    T1CONbits.TSIDL = 0; //Continues module operation in Idle mode
    T1CONbits.TCS = 0; //internal clock
    T1CONbits.TCKPS = 0b10; //prescaler : 1:64
    T1CONbits.TGATE = 0; //Gated time accumulation is disabled
    IFS0bits.T1IF = 0; //Clear flag
    IEC0bits.T1IE = 1; //Enable interrupt
    T1CONbits.TON = 1; //enable Timer1
}
void initTimer2() { //Timer 2 -> timing (delay_ms,delay_us,millis,micros)
    /*32bits mode*/
    /*TMR2 = LSB & TMR3 = MSB*/
    /*period PR2 = LSB & PR3 = MSB*/

    T2CONbits.TON = 0; //disable timer
    TMR2 = 0; // Clear timer register
    TMR3 = 0;
    /*Fp = 70Mhz -> 1tick = (100/7)ns -> 4200000000tick = 60s -> 4199999999 = 0xFA56E9FF */
    PR2 = 0xE9FF; //Period value
    PR3 = 0xFA56;
    T2CONbits.TSIDL = 0; //Continues module operation in Idle mode
    T2CONbits.TCS = 0; //internal clock
    //T2CONbits.TCKPS = 0b10; //prescaler : 1:64
    T2CONbits.TCKPS = 0b00; //prescaler : 1:1
    //T2CONbits.TCKPS = 0b01; //prescaler : 1:8
    T2CONbits.TGATE = 0; //Gated time accumulation is disabled
    T2CONbits.T32 = 1; //Timer2 & Timer 3 form a single 32-bit timer
    IFS0bits.T2IF = 0; //Clear flag
    //IEC0bits.T2IE = 1;      //Enable interrupt
    IEC0bits.T2IE = 0;

    IFS0bits.T3IF = 0; //Clear flag
    IEC0bits.T3IE = 1;

    T2CONbits.TON = 1; //enable Timer1
}
void initTimer3() { //Timer 3   -> 20µs delay
    /*T3CONbits.TON = 0;      //disable timer
    TMR3 = 0;               // Clear timer register
    PR3 = 1400;           //Period value
    T3CONbits.TSIDL = 0;    //Continues module operation in Idle mode
    T3CONbits.TCS = 0;      //internal clock
    T3CONbits.TCKPS = 0b00; //prescaler : 1:1
    T3CONbits.TGATE = 0;    //Gated time accumulation is disabled
    IFS0bits.T3IF = 0;      //Clear flag
    IEC0bits.T3IE = 1;      //Enable interrupt
    T3CONbits.TON = 0;      //disable Timer3*/
}
void initTimer4() { //Timer 4   -> count US time
    T4CONbits.TON = 0; //disable timer
    TMR4 = 0; // Clear timer register
    PR4 = 0xFFFF; //Period value
    T4CONbits.TSIDL = 0; //Continues module operation in Idle mode
    T4CONbits.TCS = 0; //internal clock
    T4CONbits.TCKPS = 0b10; //prescaler : 1:64
    T4CONbits.TGATE = 0; //Gated time accumulation is disabled
    IFS1bits.T4IF = 0; //Clear flag
    IEC1bits.T4IE = 0; //Enable interrupt
    T4CONbits.TON = 1; //enable Timer4
}
void initTimer5() { //Timer 5   -> 20µs delay
    T5CONbits.TON = 0; //disable timer
    TMR5 = 0; // Clear timer register
    PR5 = 1400; //Period value
    T5CONbits.TSIDL = 0; //Continues module operation in Idle mode
    T5CONbits.TCS = 0; //internal clock
    T5CONbits.TCKPS = 0b00; //prescaler : 1:1
    T5CONbits.TGATE = 0; //Gated time accumulation is disabled
    IFS1bits.T5IF = 0; //Clear flag
    IEC1bits.T5IE = 1; //Enable interrupt
    T5CONbits.TON = 0; //disable Timer5
}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Timer interrupts">
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
    int16_t t0 = tick0 - odoTick0;
    odoTick0 = tick0;
    int16_t t1 = tick1 - odoTick1;
    odoTick1 = tick1;
    int16_t t2 = tick2 - odoTick2;
    odoTick2 = tick2;
    
    double speed0 = t0;
    double speed1 = t1;
    double speed2 = t2;
    
    /*speed0 = speed0 * 1000;     //tick/s
    speed0 = speed0 / TICKS_PER_TURN;  //tr/s
    speed0 = speed0 * WHEEL_DIAMETER * PI;  //mm/s*/
    speed0 = speed0 * 1000 / ticksPerTurn * wheelDiameter0 * PI;
    speed1 = speed1 * 1000 / ticksPerTurn * wheelDiameter1 * PI;
    speed2 = speed2 * 1000 / ticksPerTurn * wheelDiameter2 * PI;
    
    /*equation internet : "Kinematics Control of an Omnidirectional Mobile Robot"*/
    
    double xPoint = -(2.0/3.0)*sin(theta) * speed0 -(2.0/3.0)*sin((PI/3)-theta) * speed1 + (2.0/3.0)*sin((PI/3)+theta) * speed2;
    double yPoint = (2.0/3.0)*cos(theta) * speed0 -(2.0/3.0)*cos((PI/3)-theta) * speed1 -(2.0/3.0)*cos((PI/3)+theta) * speed2;
    double thetaPoint = (1.0/(3*distanceCenterToWheel))*(speed0 + speed1 + speed2);
    
    x += xPoint / 1000;
    y += yPoint / 1000;
    theta += thetaPoint / 1000;
    
    n++;
    if (n >= N_ASSERV) {//N_asserv = 20 => T = 0.02s
        
        long double distance = sqrt((xc-x)*(xc-x)+(yc-y)*(yc-y));
        
        /*if (distance < 10){
            distance = 0;
        }*/
        
        long double commandeD = compute(&pidDistance,-distance);
        setSetPoint(&pidAngle,thetac);
        long double commandeT = compute(&pidAngle,theta);
        commandeD = commandeD;
        //commandeT = 0;
        //long double commandeX = commandeD;
        
        long double commandeX = 0;
        long double commandeY = 0;
        long double distancef = sqrt((xf-x)*(xf-x)+(yf-y)*(yf-y));
        long double alpha = 0;
        if (distancef > distanceMax){
            arrived = 0;
            //alpha = atan2l(yf-y,xf-x);
            if(distance > distanceMax2){
                alpha = atan2l(yc-y,xc-x);
                alphaDebug = alpha;
                commandeX = commandeD * cosl(alpha);
                commandeY = commandeD * sinl(alpha);
            }
        }
        else{
            arrived = 1;
        }
        commandeXDebug = commandeX;
        commandeYDebug = commandeY;
        
        long double speedSp0;
        long double speedSp1;
        long double speedSp2;

        if(modeAsserv == MODE_POSITION){
            speedSp0 = -1.0 * sin(theta) * commandeX + cos(theta)*commandeY + distanceCenterToWheel * commandeT;
            speedSp1 = -sin((PI/3) - theta) * commandeX -cos((PI/3) - theta)*commandeY + distanceCenterToWheel * commandeT;
            speedSp2 = sin((PI/3) + theta) * commandeX - cos((PI/3) + theta)*commandeY + distanceCenterToWheel * commandeT;
        }
        else if(modeAsserv == MODE_SPEED){
            speedSp0 = -1.0 * sin(theta) * receivedSpeedX + cos(theta)*receivedSpeedY + distanceCenterToWheel * receivedSpeedT;
            speedSp1 = -sin((PI/3) - theta) * receivedSpeedX -cos((PI/3) - theta)*receivedSpeedY + distanceCenterToWheel * receivedSpeedT;
            speedSp2 = sin((PI/3) + theta) * receivedSpeedX - cos((PI/3) + theta)*receivedSpeedY + distanceCenterToWheel * receivedSpeedT;
        }
        setSetPoint(&pidSpeed0,speedSp0);
        setSetPoint(&pidSpeed1,speedSp1);
        setSetPoint(&pidSpeed2,speedSp2);
        
        n = 0;
        long double s0 = (tick0 - saveTick0) * 50 * wheelDiameter0;   // % 0.02s
        saveTick0 = tick0;
        long double s1 = (tick1 - saveTick1) * 50 * wheelDiameter1;   // % 0.02s
        saveTick1 = tick1;
        long double s2 = (tick2 - saveTick2) * 50 * wheelDiameter2;   // % 0.02s
        saveTick2 = tick2;
        
        
        s0 = s0 / 1496.88;
        s1 = s1 / 1496.88;
        s2 = s2 / 1496.88;
        
        long double c0 = compute(&pidSpeed0,s0);
        long double c1 = compute(&pidSpeed1,s1);
        long double c2 = compute(&pidSpeed2,s2);
        // <editor-fold defaultstate="collapsed" desc="Limit Acc">

        /*
         * if(c0 > 0 && s0 == 0)
            c0 += 1.0;
        else if(c0 < 0 && s0 == 0)
            c0 -= 1.0;
        if(c1 > 0 && s0 == 0)
            c1 += 1.0;
        else if(c1 < 0 && s0 == 0)
            c1 -= 1.0;
        if(c2 > 0 && s0 == 0)
            c2 += 1.0;
        else if(c2 < 0 && s0 == 0)
            c2 -= 1.0;
         */
        /*if(c0 - prevc0 > ACC_MAX)
            c0 = prevc0 + ACC_MAX;
        else if(c0 - prevc0 < -ACC_MAX)
            c0 = prevc0 - ACC_MAX;
        
        if(c1 - prevc1 > ACC_MAX)
            c1 = prevc1 + ACC_MAX;
        else if(c1 - prevc1 < -ACC_MAX)
            c1 = prevc1 - ACC_MAX;
        
        if(c2 - prevc2 > ACC_MAX)
            c2 = prevc2 + ACC_MAX;
        else if(c2 - prevc2 < -ACC_MAX)
            c2 = prevc2 - ACC_MAX;*/
        

        prevc0 = c0;
        prevc1 = c1;
        prevc2 = c2;
        // </editor-fold>
        if(!stop){
            sendMotor((double)c0,(double)c1,(double)c2);
        }
        else{
            sendMotor(0,0,0);
        }
        if(arrived && modeAsserv == MODE_POSITION){
            sendMotor(0,0,0);
        }
            
        
        // <editor-fold defaultstate="collapsed" desc="Plots">
        //plot(2,(uint32_t)((int32_t)(commandeT*1000)));
        //plot(2,(uint32_t)((int32_t)(s0)));
        //plot(11, (uint32_t) ((int32_t) (pidSpeed0.setPoint * 1000)));
        /*plot(21,(uint32_t)((int32_t)(pidSpeed1.setPoint*1000)));
        plot(31,(uint32_t)((int32_t)(pidSpeed2.setPoint*1000)));*/

        //plot(12, (uint32_t) ((int32_t) (s0 * 1000)));
        /*plot(22,(uint32_t)((int32_t)(s1*1000)));
        plot(32,(uint32_t)((int32_t)(s2*1000)));*/

        /*plot(1,(uint32_t)((int32_t)(xc)));
        plot(2,(uint32_t)((int32_t)(x)));
        plot(3,(uint32_t)((int32_t)(pidDistance.debugCommande)));
        plot(4,(uint32_t)((int32_t)(alpha*1800/PI)));*/
        //plot(5,(uint32_t)((int32_t)(xf)));


        /*plot(41,(uint32_t)((int32_t)(commandeX*1000)));
        plot(42,(uint32_t)((int32_t)(commandeY*1000)));*/

        /*plot(31,(uint32_t)((int32_t)(pidAngle.setPoint*1800/PI)));
        plot(32,(uint32_t)((int32_t)(theta*1800/PI)));*/

        /*plot(41,(uint32_t)((int32_t)(PWM_0)));
        plot(42,(uint32_t)((int32_t)(PWM_1)));
        plot(43,(uint32_t)((int32_t)(PWM_2)));*/
        //sendLog("a\n");// </editor-fold>

        
        // <editor-fold defaultstate="collapsed" desc="PathGeneration">
        if (statePathGeneration != 0) {
            // <editor-fold defaultstate="collapsed" desc="Plots">

            //</editor-fold>
        }
        switch (statePathGeneration) {
            case 0:
                break;
            case 1: //Init
                statePathGeneration = 2;
                stateTrap = 1;
                xf = cx;
                yf = cy;
                tf = ct;
                y_0 = y;
                x_0 = x;
                dx = cx - x_0;
                dy = cy - y_0;
                alphaTraj = atan2(dy, dx);
                totalDistance = sqrt(dx * dx + dy * dy);
                break;
            case 2: //Translation
                switch (stateTrap) {
                    case 1: //acceleration
                        if (speed < speedMax && dist < totalDistance / 2) {
                            speed += acc * TE;
                            dist += TE * (precSpeed + speed) / 2;
                            xc = x_0 + dist * cos(alphaTraj);
                            yc = y_0 + dist * sin(alphaTraj);
                            precSpeed = speed;
                        } else {
                            stateTrap = 2;

                            dist1 = dist;
                            if (speed > speedMax)
                                speed = speedMax;
                        }
                        break;
                    case 2: //constant speed
                        if (dist < totalDistance - dist1) { //Condition
                            dist += TE * speed;
                            xc = x_0 + dist * cos(alphaTraj);
                            yc = y_0 + dist * sin(alphaTraj);
                            precSpeed = speed;
                        } else {
                            stateTrap = 3;

                            acc = -acc;
                        }
                        break;
                    case 3: //deceleration
                        if (speed > 0 && dist < totalDistance) { //Condition		//v > 0			/		d < totalDistance
                            speed += acc * TE;
                            dist += TE * (precSpeed + speed) / 2;
                            xc = x_0 + dist * cos(alphaTraj);
                            yc = y_0 + dist * sin(alphaTraj);
                            precSpeed = speed;
                        } else {
                            statePathGeneration = 0;
                            stateTrap = 1;

                            xc = cx;
                            yc = cy;

                            finalPoint = 1;

                        }
                        break;

                }
                break;
        }// </editor-fold>

        
        
    }  
}// </editor-fold>

/*void __attribute__((interrupt,no_auto_psv)) _T2Interrupt(void){
    IFS0bits.T2IF = 0; //Clear Timer1 interrupt flag
}*/
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0; //Clear Timer1 interrupt flag
    timer2Overflow++;
}

void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {// _T3Interrupt(void){
    IFS1bits.T5IF = 0;
    T5CONbits.TON = 0; //disable Timer3
    TMR5 = 0; //reset Timer3
    TRIG_US_0 = 0;
    TRIG_US_1 = 0;
    TRIG_US_2 = 0;
    TRIG_US_3 = 0;
    TRIG_US_4 = 0;
    TRIG_US_5 = 0;
}
// </editor-fold>


unsigned long micros(){
    IEC0bits.T3IE = 0;
    uint32_t saveTMR2 = TMR2;
    uint32_t ret = TMR3;
    ret = ret << 16;
    ret = ret + saveTMR2;
    ret = ret / 70;
    uint32_t t2of = timer2Overflow;
    t2of = t2of * 60000000UL;
    ret = ret + t2of;
    IEC0bits.T3IE = 1;
    return ret;
}
unsigned long millis(){
    IEC0bits.T3IE = 0;
    uint32_t saveTMR2 = TMR2;
    uint32_t ret = TMR3;
    ret = ret << 16;
    ret = ret + saveTMR2;
    ret = ret / 70000; //140MHz
    //ret = ret / 3684;   //7.3728Mhz
    uint32_t t2of = timer2Overflow;
    t2of = t2of * 60000UL;    //140Mhz
    //t2of = t2of * 3157;     //7.3728Mhz
    ret = ret + t2of;
    IEC0bits.T3IE = 1;
    return ret;
}

void delay_us(uint32_t delay){
   uint32_t tick = micros();
   while(micros() - tick < delay);
}
void delay_ms(uint32_t delay){
    uint32_t tick = millis();
    while(millis() - tick < delay);
}