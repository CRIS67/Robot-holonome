/*
 * File:   interrupt.c
 * Author: Quentin BOYER
 *
 * Created on 28 octobre 2017, 16:51
 */

#include <p33EP512GM310.h>
#include <stdint.h>
#include "interrupt.h"

extern volatile char   US_ON[NB_US];
extern volatile char   US_R[NB_US];
extern volatile double US[NB_US];

volatile int i = 0;
volatile unsigned int saveA;
volatile unsigned int saveB;
volatile unsigned int saveC;
volatile unsigned int saveD;
volatile unsigned int saveE;
volatile unsigned int saveF;
volatile unsigned int saveG;
//volatile SavePort saveG;

extern volatile unsigned char US_received;

extern volatile int16_t tick0;
extern volatile int16_t tick1;
extern volatile int16_t tick2;

void initInt() {
    INTCON1 = 0x8000;           //Disable traps and interrupt nesting
    INTCON2 = 0;                //Disable interruptions and traps and internal interrupt occurs on positive edge
    saveA = PORTA;
    saveB = PORTB;
    saveC = PORTC;
    saveD = PORTD;
    saveE = PORTE;
    saveF = PORTF;
    saveG = PORTG;
    IEC1bits.CNIE = 1;          //Enable Input Change Interrupt
    INTCON2bits.GIE = 1;        //Enable Global Interrupt
}
void _ISR_PSV _CNInterrupt(){   //Change Notification interrupt
    IFS1bits.CNIF = 0;          // Clear CN interrupt

    /*save state of inputs*/
    //uint16_t A = PORTA; 
    //uint16_t B = PORTB;
    //uint16_t C = PORTC; 
    //uint16_t D = PORTD;
    uint16_t E = PORTE;
    //uint16_t F = PORTF; 
    uint16_t G = PORTG;
    
    // <editor-fold defaultstate="collapsed" desc="COD_ASS_0">
    if ((saveG & (1 << 6)) != (G & (1 << 6))) { //COD_ASS_0A
        if (G & (1 << 6)) { //COD_ASS_0A = 1
            if (G & (1 << 7)) { //COD_ASS_0B = 1
                tick0--;
            } else {
                tick0++; //COD_ASS_0B = 0
            }
        } else { //COD_ASS_0A = 0
            if (G & (1 << 7)) { //COD_ASS_0B = 1
                tick0++;
            } else { //COD_ASS_0B = 0
                tick0--;
            }
        }
    }
    if ((saveG & (1 << 7)) != (G & (1 << 7))) { //COD_ASS_0B
        if (G & (1 << 7)) { //COD_ASS_0B = 1
            if (G & (1 << 6)) { //COD_ASS_0A = 1
                tick0++;
            } else {
                tick0--; //COD_ASS_0A = 0
            }
        } else { //COD_ASS_0B = 0
            if (G & (1 << 6)) { //COD_ASS_0A = 1
                tick0--;
            } else { //COD_ASS_0A = 0
                tick0++;
            }
        }
    }// </editor-fold>
    // <editor-fold defaultstate="collapsed" desc="COD_ASS_1">
    if ((saveG & (1 << 8)) != (G & (1 << 8))) { //COD_ASS_1A
        if (G & (1 << 8)) { //COD_ASS_1A = 1
            if (G & (1 << 9)) { //COD_ASS_1B = 1
                tick1--;
            } else {
                tick1++; //COD_ASS_1B = 0
            }
        } else { //COD_ASS_1A = 0
            if (G & (1 << 9)) { //COD_ASS_1B = 1
                tick1++;
            } else { //COD_ASS_1B = 0
                tick1--;
            }
        }
    }
    if ((saveG & (1 << 9)) != (G & (1 << 9))) { //COD_ASS_1B
        if (G & (1 << 9)) { //COD_ASS_1B = 1
            if (G & (1 << 8)) { //COD_ASS_1A = 1
                tick1++;
            } else {
                tick1--; //COD_ASS_1A = 0
            }
        } else { //COD_ASS_1B = 0
            if (G & (1 << 8)) { //COD_ASS_1A = 1
                tick1--;
            } else { //COD_ASS_1A = 0
                tick1++;
            }
        }
    }// </editor-fold>
    // <editor-fold defaultstate="collapsed" desc="COD_ASS_2">
    if ((saveG & (1 << 10)) != (G & (1 << 10))) { //COD_ASS_2A
        if (G & (1 << 10)) { //COD_ASS_0A = 1
            if (E & (1 << 8)) { //COD_ASS_0B = 1
                tick2--;
            } else {
                tick2++; //COD_ASS_0B = 0
            }
        } else { //COD_ASS_0A = 0
            if (E & (1 << 8)) { //COD_ASS_0B = 1
                tick2++;
            } else { //COD_ASS_0B = 0
                tick2--;
            }
        }
    }
    if ((saveE & (1 << 8)) != (E & (1 << 8))) { //COD_ASS_0B
        if (E & (1 << 8)) { //COD_ASS_0B = 1
            if (G & (1 << 10)) { //COD_ASS_0A = 1
                tick2++;
            } else {
                tick2--; //COD_ASS_0A = 0
            }
        } else { //COD_ASS_0B = 0
            if (G & (1 << 10)) { //COD_ASS_0A = 1
                tick2--;
            } else { //COD_ASS_0A = 0
                tick2++;
            }
        }
    }// </editor-fold>

    //saveA = A;
    //saveB = B;
    //saveC = C;
    //saveD = D;
    saveE = E;
    //saveF = F;
    saveG = G;
}
/*void _ISR_PSV _CNInterrupt(){   //Change Notification interrupt
    //print("CN_INT\n");
    IFS1bits.CNIF = 0;          // Clear CN interrupt
    pinCN p = DEFAULT;
    unsigned char n;
    
    if((saveA & (1 << 8)) != PORTAbits.RA8){
        //p = ECHO_0;
        p = ECHO;
        n = 0;
    }
    else if((saveD & (1 << 14)) != PORTDbits.RD14){
        //p = ECHO_1;
        p = ECHO;
        n = 1;
    }
    else if((saveE & (1 << 14)) != PORTEbits.RE14){
        //p = ECHO_2;
        p = ECHO;
        n = 2;
    }
    else if((saveE & (1 << 12)) != PORTEbits.RE12){
        //p = ECHO_3;
        p = ECHO;
        n = 3;
    }
    else if((saveF & (1 << 13)) != PORTFbits.RF13){
        //p = ECHO_4;
        p = ECHO;
        n = 4;
    }
    else if((saveC & (1 << 11)) != PORTCbits.RC11){
        //p = ECHO_5;
        p = ECHO;
        n = 5;
    }
    else if((saveG & (1 << 12)) != PORTGbits.RG12){
        p = RUPT_1;
    }
    else if((saveG & (1 << 13)) != PORTGbits.RG13){
        p = RUPT_2;
    }
    switch(p){
        case RUPT_1:
            //led = 1;
            break;
        case RUPT_2:
            //led = 0;
            break;
        case ECHO:
            if(US_R[n]){
                US[n] = TMR4;
                //T4CONbits.TON = 0;      //enable Timer4
                US_R[n] = 0;
                
                US_received = 1;
            }
            if(US_ON[n]){
                TMR4 = 0;
                //T4CONbits.TON = 1;      //enable Timer4
                US_ON[n] = 0;
                US_R[n] = 1;
            }
            break;
        default:
            break;
    }
    saveA = PORTA;
    saveB = PORTB;
    saveC = PORTC;
    saveD = PORTD;
    saveE = PORTE;
    saveF = PORTF;
    saveG = PORTG;
}*/