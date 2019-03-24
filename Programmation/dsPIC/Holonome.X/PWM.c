/*
 * File:   PWM.c
 * Author: Quentin BOYER
 *
 * Note : WARNING : PWMLOCK configuration bit must be clear (or add unlock sequences)
 * Created on 19 septembre 2017, 23:34
 */

#include <p33EP512GM310.h>

#include "PWM.h"

extern uint8_t finalPoint;

void initPWM(){
    //initPWM1();
    initPWM2();
    //SENS_ACT_0 = 0;
    initPWM3();
    //initPWM4();
    //initPWM5();
    //initPWM6();
    //PTPER = 7000;   //PRIMARY MASTER TIME BASE PERIOD REGISTER
    //STPER = 43750;      //SECONDARY MASTER TIME BASE PERIOD
    STCON2bits.PCLKDIV = 0b110; //Divide-by-64          SECONDARY MASTER CLOCK DIVIDER
    PTCON2bits.PCLKDIV = 0; //Divide-by-1, maximum PWMx timing resolution
    PTCONbits.PTEN = 1; //PWMx module is enabled
    /*
    servoUs(0,1525);
    servoUs(1,800);
    servoUs(2,2300);
    servoUs(3,1650);
    servoUs(4,1500);
    servoUs(5,1900);
    servoUs(6,900);
    */
}

void initPWM1(){           //50Hz for serovomotors
    PHASE1 = 43750;         //Period of PWM1H     Fosc / (F_PWM*PWM_Input_Clock_Prescaler) ex : 140Mhz / 64*(50Hz * 1) = 43750
    SPHASE1 = 43750;        //Period of PWM1L
    PDC1 = 0;               //Duty cycle of PWM1H
    SDC1 = 0;               //Duty cycle of PWM1L
    IOCON1bits.PENH = 1;    //PWMx module controls the PWMxH pin
    IOCON1bits.PENL = 1;    //PWMx module controls the PWMxL pin
    IOCON1bits.PMOD = 0b11; //PWMx I/O pin pair is in the True Independent Output mode
    PWMCON1bits.ITB = 1;    //PHASEx register provides the time base period for this PWMx generator
    PWMCON1bits.MTBS = 1;   //PWMx generator uses the secondary master time base (1/64)
    FCLCON1 = 0x0003;       //Fault input is disabled
}
void initPWM2(){           //20kHz
    PHASE2 = 7000;         //Period of PWM2H     Fosc / (F_PWM*PWM_Input_Clock_Prescaler) ex : 140Mhz / 64*(50Hz * 1) = 43750
    SPHASE2 = 7000;        //Period of PWM2L
    PDC2 = 0;               //Duty cycle of PWM2H
    SDC2 = 0;               //Duty cycle of PWM2L
    //IOCON2bits.PENH = 1;    //PWMx module controls the PWMxH pin
    //IOCON2bits.PENL = 1;    //PWMx module controls the PWMxL pin
    IOCON2bits.PENH = 0;    //PWMx module controls the PWMxH pin
    IOCON2bits.PENL = 0;    //PWMx module controls the PWMxL pin
    IOCON2bits.PMOD = 0b11; //PWMx I/O pin pair is in the True Independent Output mode
    PWMCON2bits.ITB = 1;    //PHASEx register provides the time base period for this PWMx generator
    PWMCON2bits.MTBS = 0;   //PWMx generator uses the primary master time base  (1/1)
    FCLCON2 = 0x0003;       //Fault input is disabled
    SENS_ACT_0 = 1;
    SENS_ACT_1 = 1;
    LATBbits.LATB12 = 0;
    LATBbits.LATB13 = 0;
}
void initPWM3(){           //20kHz
    PHASE3 = 7000;         //Period of PWM3H     Fosc / (F_PWM*PWM_Input_Clock_Prescaler) ex : 140Mhz / 64*(50Hz * 1) = 43750
    SPHASE3 = 7000;        //Period of PWM3L
    PDC3 = 0;               //Duty cycle of PWM3H
    SDC3 = 0;               //Duty cycle of PWM3L
    IOCON3bits.PENH = 1;    //PWMx module controls the PWMxH pin
    IOCON3bits.PENL = 1;    //PWMx module controls the PWMxL pin
    IOCON3bits.PMOD = 0b11; //PWMx I/O pin pair is in the True Independent Output mode
    PWMCON3bits.ITB = 1;    //PHASEx register provides the time base period for this PWMx generator
    PWMCON3bits.MTBS = 0;   //PWMx generator uses the primary master time base  (1/1)
    FCLCON3 = 0x0003;       //Fault input is disabled
}
void initPWM4(){           //50Hz for serovomotors
    PHASE4 = 109;           //Period of PWM4H     Fosc / (F_PWM*PWM_Input_Clock_Prescaler) ex : 140Mhz / 64*(50Hz * 1) = 43750
    SPHASE4 = 43750;        //Period of PWM4L
    PDC4 = 0;               //Duty cycle of PWM4H
    SDC4 = 0;               //Duty cycle of PWM4L
    IOCON4bits.PENH = 1;    //PWMx module controls the PWMxH pin
    IOCON4bits.PENL = 1;    //PWMx module controls the PWMxL pin
    IOCON4bits.PMOD = 0b11; //PWMx I/O pin pair is in the True Independent Output mode
    PWMCON4bits.ITB = 1;    //PHASEx register provides the time base period for this PWMx generator
    PWMCON4bits.MTBS = 1;   //PWMx generator uses the secondary master time base (1/64)
    FCLCON4 = 0x0003;       //Fault input is disabled
}
void initPWM5(){       //50Hz for serovomotors
    PHASE5 = 43750;      //Period of PWM5H     Fosc / (F_PWM*PWM_Input_Clock_Prescaler) ex : 140Mhz / (20kHz * 1) = 7000 or wih Fosc = 138.24MHz -> 6912
    SPHASE5 = 43750;     //Period of PWM5L
    PDC5 = 0;           //Duty cycle of PWM5H
    SDC5 = 0;           //Duty cycle of PWM5L
    //DTR5 = 0;           //Dead Time
    //ALTDTR5 = 0;
    IOCON5bits.PENH = 1;    //PWMx module controls the PWMxH pin
    IOCON5bits.PENL = 1;    //PWMx module controls the PWMxL pin
    IOCON5bits.PMOD = 0b11; //PWMx I/O pin pair is in the True Independent Output mode
    PWMCON5bits.ITB = 1;    //PHASEx register provides the time base period for this PWMx generator
    PWMCON5bits.MTBS = 1;   //PWMx generator uses the secondary master time base for synchronization and as the clock source for the PWMx generation logic
    FCLCON5 = 0x0003;
}
void initPWM6(){           //50Hz for serovomotors
    PHASE6 = 43750;         //Period of PWM6H     Fosc / (F_PWM*PWM_Input_Clock_Prescaler) ex : 140Mhz / 64*(50Hz * 1) = 43750
    SPHASE6 = 43750;        //Period of PWM6L
    PDC6 = 0;               //Duty cycle of PWM6H
    SDC6 = 0;               //Duty cycle of PWM6L
    IOCON6bits.PENH = 1;    //PWMx module controls the PWMxH pin
    IOCON6bits.PENL = 1;    //PWMx module controls the PWMxL pin
    IOCON6bits.PMOD = 0b11; //PWMx I/O pin pair is in the True Independent Output mode
    PWMCON6bits.ITB = 1;    //PHASEx register provides the time base period for this PWMx generator
    PWMCON6bits.MTBS = 1;   //PWMx generator uses the secondary master time base for synchronization and as the clock source for the PWMx generation logic
    FCLCON6 = 0x0003;       //Fault input is disabled
}

void motor(uint8_t id, char value){
    double dval = (double)value;
    dval = dval / 12.5; // [-100;100]% -> [-8;8]V
    switch(id){
        case 0:
            sendMotor0(dval);
            break;
        case 1:
            sendMotor1(dval);
            break;
        case 2:
            sendMotor2(dval);
            break;
        default:
            break;
    }
}
void sendMotor0(double value){
    if(value <= -VSAT){
        SENS_0 = BACKWARD;
        PWM_0 = PWM_PR_0 * (VSAT / VBAT);
    }
    else if(value >= VSAT){
        SENS_0 = FORWARD;
        PWM_0 = PWM_PR_0 * (VSAT / VBAT);
    }
    else if(value < 0){
        SENS_0 = BACKWARD;
        PWM_0 = -(int)(value * PWM_PR_0 / VBAT);
    }
    else{
        SENS_0 = FORWARD;
        PWM_0 = (int)(value * PWM_PR_0 / VBAT);
    }
}
void sendMotor1(double value){
    if(value <= -VSAT){
        SENS_1 = BACKWARD;
        PWM_1 = PWM_PR_1 * (VSAT / VBAT);
    }
    else if(value >= VSAT){
        SENS_1 = FORWARD;
        PWM_1 = PWM_PR_1 * (VSAT / VBAT);
    }
    else if(value < 0){
        SENS_1 = BACKWARD;
        PWM_1 = -(int)(value * PWM_PR_1 / VBAT);
    }
    else{
        SENS_1 = FORWARD;
        PWM_1 = (int)(value * PWM_PR_1 / VBAT);
    }
}
void sendMotor2(double value){
    if(value <= -VSAT){
        SENS_2 = BACKWARD;
        PWM_2 = PWM_PR_2 * (VSAT / VBAT);
    }
    else if(value >= VSAT){
        SENS_2 = FORWARD;
        PWM_2 = PWM_PR_2 * (VSAT / VBAT);
    }
    else if(value < 0){
        SENS_2 = BACKWARD;
        PWM_2 = -(int)(value * PWM_PR_2 / VBAT);
    }
    else{
        SENS_2 = FORWARD;
        PWM_2 = (int)(value * PWM_PR_2 / VBAT);
    }
}
void sendMotor(double val0, double val1, double val2){
    sendMotor0(val0);
    sendMotor1(val1);
    sendMotor2(val2);
}
