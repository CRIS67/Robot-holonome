/*
 * File:   GPIO.c
 * Author: Quentin BOYER
 *
 * Created on September 18, 2017, 11:23 PM
 */

#include <p33EP512GM310.h>

#include "GPIO.h"
#include "interrupt.h"

void initGPIO() {
    
    //1 -> input / 0 -> output
    /*
    TRISA = 0xFFFF;
    TRISB = 0xFFFF;
    TRISC = 0xFFFF;
    TRISD = 0xFFFF;
    TRISE = 0xFFFF;
    TRISF = 0xFFFF;
    TRISG = 0xFFFF;
    */
    
    TRISBbits.TRISB5 = 1;   //RX_BT
    TRISBbits.TRISB6 = 0;   //TX_BT
    TRISBbits.TRISB7 = 1;   //I_ASS_2
    TRISBbits.TRISB10 = 0;  //PWM_ASS_0
    TRISBbits.TRISB11 = 0;  //PWM_ASS_1
    TRISBbits.TRISB13 = 0;  //PWM_ASS_2
    
    TRISCbits.TRISC4 = 1;   //MES_BAT
    TRISCbits.TRISC6 = 1;   //RX_ATMEGA
    TRISCbits.TRISC7 = 0;   //TX_ATMEGA
    
    TRISEbits.TRISE0 = 1;   //RX_RPI
    TRISEbits.TRISE1 = 0;   //TX_RPI
    TRISEbits.TRISE8 = 1;   //COD_ASS_2B
    TRISEbits.TRISE9 = 1;   //I_ASS_1
    
    TRISFbits.TRISF4 = 0;   //LED_1
    TRISFbits.TRISF5 = 0;   //LED_2
    
    TRISGbits.TRISG0 = 0;   //SENS_ASS_0
    TRISGbits.TRISG1 = 0;   //SENS_ASS_1
    TRISGbits.TRISG3 = 0;   //LED_0
    TRISGbits.TRISG6 = 1;   //COD_ASS_0A
    TRISGbits.TRISG7 = 1;   //COD_ASS_0B
    TRISGbits.TRISG8 = 1;   //COD_ASS_1A
    TRISGbits.TRISG9 = 1;   //COD_ASS_1B
    TRISGbits.TRISG10 = 1;   //COD_ASS_2A
    TRISGbits.TRISG12 = 0;   //SENS_ASS_2
    TRISGbits.TRISG15 = 1;   //I_ASS_2
    
    
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    ANSELC = 0x0000;
    ANSELD = 0x0000;
    ANSELE = 0x0000;
    ANSELF = 0x0000;
    ANSELG = 0x0000;
     
    /*ANSELAbits.ANSA4 = 1;   //MESURE_BAT
    ANSELBbits.ANSB8 = 1;   //COURANT_ASS_ 0
    ANSELBbits.ANSB9 = 1;   //COURANT_ASS_ 1*/
    
    ANSELBbits.ANSB7 = 1;   //I_ASS_0 - AN25
    ANSELCbits.ANSC4 = 1;   //MES_BAT - AN30
    ANSELEbits.ANSE9 = 1;   //I_ASS_1 - AN20
    ANSELGbits.ANSG15 = 1;   //I_ASS_0 - AN23

    //1 -> open-drain
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;
    ODCE = 0x0000;
    ODCF = 0x0000;
    ODCG = 0x0000;
    //1 -> pull-up resistor
    CNPUA = 0x0000;
    CNPUB = 0x0000;
    CNPUC = 0x0000;
    CNPUD = 0x0000;
    CNPUE = 0x0000;
    CNPUF = 0x0000;
    CNPUG = 0x0000;
    
    /*CNPUAbits.CNPUA8 = 1;   //ECHO_0
    CNPUDbits.CNPUD14 = 1;  //ECHO_1
    CNPUEbits.CNPUE14 = 1;  //ECHO_2
    CNPUEbits.CNPUE12 = 1;  //ECHO_3
    CNPUFbits.CNPUF13 = 1;  //ECHO_4
    CNPUCbits.CNPUC11 = 1;  //ECHO_5*/
    
    //1 -> pull-down resistor
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPDC = 0x0000;
    CNPDD = 0x0000;
    CNPDE = 0x0000;
    CNPDF = 0x0000;
    CNPDG = 0x0000;
    /*
    CNPDAbits.CNPDA8 = 1;   //ECHO_0
    CNPDDbits.CNPDD14 = 1;  //ECHO_1
    CNPDEbits.CNPDE14 = 1;  //ECHO_2
    CNPDEbits.CNPDE12 = 1;  //ECHO_3
    CNPDFbits.CNPDF13 = 1;  //ECHO_4
    CNPDCbits.CNPDC11 = 1;  //ECHO_5
    */
    //1 -> genere an interrupt request on a Change-of-State
    CNENA = 0x0000;
    CNENB = 0x0000;
    CNENC = 0x0000;
    CNEND = 0x0000;
    CNENE = 0x0000;
    CNENF = 0x0000;
    CNENG = 0x0000;
    /*
    CNENAbits.CNIEA8 = 1;
    CNENDbits.CNIED14 = 1;
    CNENEbits.CNIEE14 = 1;
    CNENEbits.CNIEE12 = 1;
    CNENFbits.CNIEF13 = 1;
    CNENCbits.CNIEC11 = 1;
    */
    /*
#define ECHO_US_0   PORTAbits.RA8
#define ECHO_US_1   PORTDbits.RD14
#define ECHO_US_2   PORTEbits.RE14
#define ECHO_US_3   PORTEbits.RE12
#define ECHO_US_4   PORTFbits.RF13
#define ECHO_US_5   PORTCbits.RC11
 */
    //CNENGbits.CNIEG12 = 1;
    //CNENGbits.CNIEG13 = 1;
    //1 -> High(3.3V) or 0V(open-drain) / 0 -> Low(0V) or high-impedance(open-drain)
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;
    LATD = 0x0000;
    LATE = 0x0000;
    LATF = 0x0000;
    LATG = 0x0000;
    
    /*Peripheral Pin Select (PPS)*/
    //QEI
	/*
    RPINR14bits.QEA1R = 0b1110110;  //QEI1A -> RPI118 (pin10)
    RPINR14bits.QEB1R = 0b1110111;  //QEI1B -> RPI119 (pin11)
    
    RPINR16bits.QEA2R = 0b1111000;  //QEI2A -> RPI120 (pin12)
    RPINR16bits.QEB2R = 0b1111001;  //QEI2B -> RPI121 (pin14)
    */
}