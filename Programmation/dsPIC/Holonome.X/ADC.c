/*
 * File:   ADC.c
 * Author: Quentin BOYER
 *
 * Created on 22 septembre 2017, 21:36
 */

#include "ADC.h"

void initADC() {
    initADC1();
}

void initADC1(){
    AD1CON1bits.ADON = 0;   //Turn off the module
    AD1CON1bits.DONE = 0;
    AD1CON1bits.SAMP = 0;
    AD1CON1bits.ASAM = AUTO_SAMPLE;
    AD1CON1bits.SIMSAM = 0; //Samples multiple channels individually in sequence
    AD1CON1bits.SSRCG = 0;
    AD1CON1bits.SSRC = 0b111;   //Internal counter ends sampling and starts conversion (auto-convert)
    AD1CON1bits.FORM = 0b00;    //Signed integer (DOUT = ssss sssd dddd dddd, where s = .NOT.d<9>)
    AD1CON1bits.AD12B = 0;      //10-bit, 4-channel ADCx operation
    AD1CON1bits.ADDMABM = 0;    //DMA buffers are written in Scatter/Gather mode; the module provides a Scatter/Gather address to the DMA channel based on the index of the analog input and the size of the DMA buffer
    AD1CON1bits.ADSIDL = 0;     //Continues module operation in Idle mode
    
    AD1CON2bits.ALTS = 0;       //Always uses channel input selects for Sample MUXA
    AD1CON2bits.BUFM = 0;       //Always starts filling the buffer from the Start address
    AD1CON2bits.SMPI = 0b00000; //Generates interrupt after completion of every sample/conversion operation
    AD1CON2bits.CHPS = 0b00;    //Converts CH0
    AD1CON2bits.CSCNA = 0;      //Does not scan inputs
    //AD1CON2bits.OFFCAL = 0;     //+ and ? inputs of channel Sample-and-Hold are normal
    AD1CON2bits.VCFG = 0b000;   //AVdd and Avss
    
    AD1CON3bits.ADCS = 0b01110111;  //TP ? (ADCS<7:0> + 1) = TP ? 120 = TAD
    AD1CON3bits.SAMC = 0b00100; //Auto-Sample Time bits = 5 TAD
    AD1CON3bits.ADRC = 0;       //Clock derived from system clock
    
    AD1CON4bits.DMABL = 0b000;  //Allocates 1 word of buffer to each analog input
    AD1CON4bits.ADDMAEN = 0;    //Conversion results are stored in the ADC1BUF0 through ADC1BUFF registers; DMA will not be used
    
    AD1CHS123 = 0;  
    //AD1CHS0 = 0;
    AD1CHS0bits.CH0NB = 0;
    //AD1CHS0bits.CH0SB = 0b011000;//AN24
    AD1CHS0bits.CH0SB = 30; //AN30
    AD1CHS0bits.CH0NA = 0;
    //AD1CHS0bits.CH0SA = 0b011000;//AN24
    AD1CHS0bits.CH0SA = 30; //AN30
    AD1CSSL = 0;
    AD1CSSH = 0;
    
    AD1CON1bits.ADON = 1;
}

int readADC1(){
    if(!AUTO_SAMPLE){
    AD1CSSLbits.CSS0 = 1;
    AD1CON1bits.DONE = 0;
    AD1CON1bits.SAMP = 1;
    }
    while(!AD1CON1bits.DONE);
    return ADC1BUF0;
}