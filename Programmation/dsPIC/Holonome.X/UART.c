/*
 * File:   UART.c
 * Author: Quentin BOYER
 *
 * Created on 31 octobre 2017, 21:10
 * 
 * Notes : Parfois des erreurs lors de l'envoi de plusieurs octets en meme temps depui la RPi, � v�rifier en augmentant la clock
 */

#include "UART.h"
#include "PWM.h"

// <editor-fold defaultstate="collapsed" desc="Variables">
//extern long double xc;
extern int state;
extern PID pidAngle, pidDistance, pidSpeedLeft, pidSpeedRight;
extern volatile PID pidSpeed0, pidSpeed1, pidSpeed2;
extern int R, L;
extern volatile double US[NB_US];
extern volatile long double x;
extern volatile long double y;
extern volatile long double theta;

extern volatile long double xc;
extern volatile long double yc;
extern volatile long double thetac;

extern volatile long double xf;
extern volatile long double yf;
extern volatile long double tf;

extern volatile unsigned char stop;

extern volatile uint8_t verbose;

char TxLoopBuffer[TX_SIZE];
char TxLoopBuffer2[TX_SIZE];
char RxBuffer[RX_SIZE];
char RxBuffer2[RX_SIZE];
volatile uint8_t RxDMABuffer[RX_DMA_SIZE];
//unsigned char iD,iF,iD2,iF2;                //index of first data and of first free element
uint16_t iD, iF, iD2, iF2; //index of first data and of first free element
unsigned char TxOn, TxOn2;
//unsigned char iRx,iRx2;
uint16_t iRx, iRx2;

volatile double receivedX, receivedY, receivedTheta;
volatile unsigned char newPosReceived = 0;
volatile unsigned char newPosBackReceived = 0;
volatile unsigned char newAngleReceived = 0;

volatile unsigned char debugPosRpi = 0;
volatile unsigned char debugPosRpiAsserv = 0;

uint16_t start = 0;

uint32_t testPID = 0; 

extern long double coef_dissymmetry;
extern long double mm_per_ticks;
extern long double rad_per_ticks;
extern long double distance_between_encoder_wheels;


extern volatile long double ticksPerTurn;
extern volatile long double wheelDiameter;
extern volatile long double distanceCenterToWheel;


extern volatile long double wheelDiameter0;
extern volatile long double wheelDiameter1;
extern volatile long double wheelDiameter2;


extern volatile double funSpeed;
extern volatile double funAcc;

extern volatile double funAngularSpeed;
extern volatile double funAngularAcc;

extern volatile long double linSpeed;  //mm/s
extern volatile long double linAcc;    //mm/s^2
extern volatile long double rotSpeed;  //rad/s
extern volatile long double rotAcc;    //rad/s^2

extern volatile uint8_t arrived;
extern volatile long double distanceMax;

extern long double receivedSpeedX;
extern long double receivedSpeedY;
extern long double receivedSpeedT;
extern uint8_t modeAsserv;
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="Init">

void initUART() {
    //initUART1();
    initUART2();
}

void initUART1() { //Bluetooth
    IEC0bits.U1TXIE = 0; //Disable UART1 Tx interrupt
    IEC0bits.U1RXIE = 0; //Disable UART1 Rx interrupt

    TRISBbits.TRISB6 = 0; //TX
    TRISBbits.TRISB5 = 1; //RX

    RPINR18 = 0b0100101;
    RPOR2bits.RP38R = 0b000001;

    U1MODEbits.UARTEN = 0;
    U1MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U1MODEbits.IREN = 0; // Bit12 No IR translation
    U1MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    U1MODEbits.UEN = 0b00;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.URXINV = 0;
    U1MODEbits.BRGH = 0;
    U1MODEbits.PDSEL = 0b00;
    U1MODEbits.STSEL = 0;

    U1STAbits.UTXBRK = 0; //Bit11 Disabled
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0; //Interrupt is generated when any character is transferred to the Transmit Shift Register and the transmit buffer is empty (which implies at least one location is empty in the transmit buffer)
    U1STAbits.UTXINV = 0;
    U1STAbits.ADDEN = 0;
    U1STAbits.URXISEL = 0;

    U1BRG = BRGVAL;

    IFS0bits.U1TXIF = 0; // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 1; // Enable Transmit Interrupts
    IFS0bits.U1RXIF = 0; // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 1; // Enable Recieve Interrupts

    U1MODEbits.UARTEN = 1; //Enable the module
    U1STAbits.UTXEN = 1;

    iD = 0;
    iF = 0;
    TxOn = 0;
    iRx = 0;
}

void initUART2() { //Raspberry Pi
    IEC1bits.U2TXIE = 0; //Disable UART2 Tx interrupt
    IEC1bits.U2RXIE = 0; //Disable UART2 Rx interrupt

    RPINR19 = 0b01010000; //RPI80(pin52-RE0) tied to UART2 RX
    RPOR9bits.RP81R = 0b000011; //RP81 (pin53-RE1) tied to UART2 TX

    U2MODEbits.UARTEN = 0;
    U2MODEbits.USIDL = 0; // Bit13 Continue in Idle
    U2MODEbits.IREN = 0; // Bit12 No IR translation
    U2MODEbits.RTSMD = 0; // Bit11 Simplex Mode
    U2MODEbits.UEN = 0b00;
    U2MODEbits.LPBACK = 0;
    U2MODEbits.ABAUD = 0;
    U2MODEbits.URXINV = 0;
    U2MODEbits.BRGH = UART2_HIGH_SPEED;
    U2MODEbits.PDSEL = 0b00;
    U2MODEbits.STSEL = 0;

    U2STAbits.UTXBRK = 0; //Bit11 Disabled
    U2STAbits.UTXISEL0 = 0;
    U2STAbits.UTXISEL1 = 0; //Interrupt is generated when any character is transferred to the Transmit Shift Register and the transmit buffer is empty (which implies at least one location is empty in the transmit buffer)
    U2STAbits.UTXINV = 0;
    U2STAbits.ADDEN = 0;
    U2STAbits.URXISEL = 0;

    U2BRG = BRGVAL2;

    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2TXIE = 1;
    IEC1bits.U2RXIE = 0;

    U2MODEbits.UARTEN = 1; //Enable the module
    U2STAbits.UTXEN = 1;

    iD2 = 0;
    iF2 = 0;
    TxOn2 = 0;
    iRx2 = 0;
}// </editor-fold>


// <editor-fold defaultstate="collapsed" desc="Interrupts">
/*void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
    IFS0bits.U1RXIF = 0;
}*/

/*void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void){
    IFS1bits.U2RXIF = 0;
}*/

/*void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    if(iD != iF){
        U1TXREG = pop();
    }
    else
        TxOn = 0;
}*/

void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void) {
    IFS1bits.U2TXIF = 0; // Clear TX Interrupt flag
    if (iD2 != iF2) {
        U2TXREG = pop2();
    } else
        TxOn2 = 0;
}// </editor-fold>


void push(char c){
    TxLoopBuffer[iF] = c;
    iF++;
    if(iF == TX_SIZE)
        iF = 0;
}
void push2(char c){
    TxLoopBuffer2[iF2] = c;
    iF2++;
    if(iF2 == TX_SIZE)
        iF2 = 0;
}
char pop(){
    char r = TxLoopBuffer[iD];
    iD++;
    if(iD == TX_SIZE)
        iD = 0;
    return r;
}
char pop2(){
    char r = TxLoopBuffer2[iD2];
    iD2++;
    if(iD2 == TX_SIZE)
        iD2 = 0;
    return r;
}

char *itoa(int value) {
     static char buffer[12];        // 12 bytes is big enough for an INT32
     int original = value;        // save original value
 
     int c = sizeof(buffer)-1;
 
     buffer[c] = 0;                // write trailing null in last byte of buffer    
 
     if (value < 0)                 // if it's negative, note that and take the absolute value
         value = -value;
     
     do                             // write least significant digit of value that's left
     {
         buffer[--c] = (value % 10) + '0';    
         value /= 10;
     } while (value);
 
     if (original < 0) 
         buffer[--c] = '-';
 
     return &buffer[c];
 }
char *dtoa(double value){
    static char buffer[10];
    char tempBuffer[10];
    /*int*/
    int8_t i;
    int8_t j = 0;
    int integerPart = (int)value;
    double dec = value;
    if(value < 0){
        integerPart = -integerPart;
        dec = -dec;
        buffer[0] = '-';
        j++;
    }
    dec = dec - integerPart;
    for(i = 0; i < 10; i++){
        tempBuffer[i] = (integerPart % 10) + '0';
        integerPart /= 10;
        if(!integerPart)
            break;
    }

    /*reverse string*/
    while(i >= 0){
        buffer[j] = tempBuffer[i];
        i--;
        j++;
    }
    /*decimal*/
    buffer[j] = '.';
    j++;
    while(j < 9){
        dec *= 10;
        buffer[j] = ((int)dec)%10 + '0';
        dec = dec - (int)dec;
        j++;
    }
    buffer[9] = '\0';
    return buffer;
}
void CheckMessages(){
    while(1){
        
        // <editor-fold defaultstate="collapsed" desc="Get message">
        uint16_t nextDMAWriteAdress = DMA1STAL;
        uint16_t writeIndex = nextDMAWriteAdress - (uint16_t) & RxDMABuffer;
        int16_t nBytesToRead = writeIndex - start;

        if (nBytesToRead < 0) //RX Buffer overflow
            nBytesToRead += RX_DMA_SIZE;

        if (nBytesToRead == 0) //no data in the buffer
            return;

        uint8_t size = RxDMABuffer[start];
        if (nBytesToRead < size + 1) //the message is incomplete
            return;

        /*Circular buffer index*/
        uint16_t iCode = (start + 1) % RX_DMA_SIZE;
        uint16_t iArg1 = (start + 2) % RX_DMA_SIZE;
        uint16_t iArg2 = (start + 3) % RX_DMA_SIZE;
        uint16_t iArg3 = (start + 4) % RX_DMA_SIZE;
        uint16_t iArg4 = (start + 5) % RX_DMA_SIZE;
        uint16_t iArg5 = (start + 6) % RX_DMA_SIZE;
        uint16_t iArg6 = (start + 7) % RX_DMA_SIZE;
        uint16_t iArg7 = (start + 8) % RX_DMA_SIZE;
        uint16_t iArg8 = (start + 9) % RX_DMA_SIZE;
        uint16_t iArg9 = (start + 10) % RX_DMA_SIZE;
        uint16_t iArg10 = (start + 11) % RX_DMA_SIZE;
        uint16_t iChecksum = (start + size) % RX_DMA_SIZE;

        uint16_t i, j;
        uint8_t checksum = 0;
        for (i = 0; i < size; i++) {
            j = (start + i) % RX_DMA_SIZE;
            checksum += RxDMABuffer[j];
        }
        start += RxDMABuffer[start] + 1;
        start = start % RX_DMA_SIZE;
        if (checksum != RxDMABuffer[iChecksum]) {
            uint8_t saveVerbose = verbose;
            verbose = 1;
            sendLog("Checksum error !\n");
            verbose = saveVerbose;
            return;
        }
        unsigned char success = 0; // </editor-fold>

        switch (RxDMABuffer[iCode]) {
                // <editor-fold defaultstate="collapsed" desc="Start">
            case RX_CODE_START:
                if (size != RX_SIZE_START)
                    return;
                stop = 0;
                //cout << "Start" << endl;
                sendLog(("Started !\n"));
                success = 1;
                break; // </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Stop">
            case RX_CODE_STOP:
                if (size != RX_SIZE_STOP)
                    return;
                stop = 1;
                //cout << "Stop" << endl;
                sendLog(("Stopped !\n"));
                success = 1;
                break; // </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Set">
            case RX_CODE_SET:
            {
                uint8_t var = RxDMABuffer[iArg1];
                uint8_t type = RxDMABuffer[iArg2];
                // <editor-fold defaultstate="collapsed" desc="8bits">
                if (type == VAR_8b) {
                    uint8_t value = RxDMABuffer[iArg3];
                    switch (var) {
                            /*case VAR_STATE:
                                state = value;
                                break;*/
                        case CODE_VAR_VERBOSE:
                            verbose = value;
                            verbose = 1;
                            sendLog("verbose set to ");
                            sendLog(itoa((int) value));
                            verbose = value;
                            break;
                        case CODE_VAR_MODE_ASSERV:
                            modeAsserv = value;
                            sendLog("modeAsserv set to ");
                            sendLog(itoa((int) value));
                            break;
                    }

                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="16bits">
                else if (type == VAR_16b) {
                    int16_t value = (RxDMABuffer[iArg3] << 8) + RxDMABuffer[iArg4]; //or ptr / union / memcpy
                    switch (var) {
                        case VAR_STATE:
                            sendLog("set state ");
                            sendLog(itoa(value));
                            sendLog("\n");
                            state = value;
                            break;
                    }
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="32bits">
                else if (type == VAR_32b) {
                    uint32_t value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                    switch (var) {
                        case CODE_VAR_P_SPEED_L:
                            pidSpeed0.Kp = (double) value / COEF_SCALE_PID;
                            sendLog("Changed P1 : speed left to ");
                            //sendLog(itoa((int)(pidSpeedLeft.Kp*COEF_SCALE_PID)));
                            sendLog(dtoa(pidSpeed0.Kp));
                            sendLog("\n");
                            //sendLog(" / 1000\n");
                            break;
                        case CODE_VAR_I_SPEED_L:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidSpeed0.Ki = (double) value / COEF_SCALE_PID;
                            sendLog("Changed I1 : speed left to ");
                            //sendLog(itoa((int)(pidSpeedLeft.Ki*COEF_SCALE_PID)));
                            sendLog(dtoa(pidSpeed0.Ki));
                            sendLog("\n");
                            //sendLog(" / 1000\n");
                            break;
                        case CODE_VAR_D_SPEED_L:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidSpeed0.Kd = (double) value / COEF_SCALE_PID;
                            sendLog("Changed D1 : speed left to ");
                            //sendLog(itoa((int)(pidSpeedLeft.Kd*1000)));
                            sendLog(dtoa(pidSpeed0.Kd));
                            sendLog("\n");
                            //sendLog(" / 1000\n");
                            break;
                        case CODE_VAR_P_SPEED_R:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidSpeed1.Kp = (double) value / COEF_SCALE_PID;
                            sendLog("Changed P2 : speed right to ");
                            sendLog(dtoa(pidSpeed1.Kp));
                            sendLog("\n");
                            break;
                        case CODE_VAR_I_SPEED_R:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidSpeed1.Ki = (double) value / COEF_SCALE_PID;
                            sendLog("Changed I2 : speed right to ");
                            sendLog(dtoa(pidSpeed1.Ki));
                            sendLog("\n");
                            break;
                        case CODE_VAR_D_SPEED_R:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidSpeed1.Kd = (double) value / COEF_SCALE_PID;
                            sendLog("Changed D2 : speed right to ");
                            sendLog(dtoa(pidSpeed1.Kd));
                            sendLog("\n");
                            break;
                        case CODE_VAR_P_DISTANCE:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidSpeed2.Kp = (double) value / COEF_SCALE_PID;
                            break;
                        case CODE_VAR_I_DISTANCE:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidSpeed2.Ki = (double) value / COEF_SCALE_PID;
                            break;
                        case CODE_VAR_D_DISTANCE:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidSpeed2.Kd = (double) value / COEF_SCALE_PID;
                            break;
                        case CODE_VAR_P_ANGLE:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidAngle.Kp = (double) value / COEF_SCALE_PID;
                            break;
                        case CODE_VAR_I_ANGLE:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidAngle.Ki = (double) value / COEF_SCALE_PID;
                            break;
                        case CODE_VAR_D_ANGLE:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            pidAngle.Kd = (double) value / COEF_SCALE_PID;
                            break;
                        case CODE_VAR_COEF_DISSYMETRY:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            coef_dissymmetry = (long double) value / COEF_SCALE_COEF_DISSYMETRY;
                            sendLog("Coef Dissymetry changed to : ");
                            sendLog(dtoa((double) coef_dissymmetry));
                            sendLog("\n");
                            break;
                        case CODE_VAR_MM_PER_TICKS:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            mm_per_ticks = (long double) value / COEF_SCALE_MM_PER_TICKS;
                            sendLog("mm per ticks changed to : ");
                            sendLog(dtoa((double) mm_per_ticks));
                            sendLog("\n");
                            /*funAngularSpeed = (double)(value/1000);
                            sendLog("funSpeed changed to : ");
                            sendLog(dtoa(funAngularSpeed));
                            sendLog("\n");*/
                            break;
                        case CODE_VAR_DISTANCE_BETWEEN_ENCODER_WHEELS:
                            value = ((uint32_t) RxDMABuffer[iArg3] << 24) + ((uint32_t) RxDMABuffer[iArg4] << 16) + ((uint32_t) RxDMABuffer[iArg5] << 8) + RxDMABuffer[iArg6];
                            distance_between_encoder_wheels = (long double) value / COEF_SCALE_DISTANCE_BETWEEN_ENCODER_WHEELS;
                            sendLog("Distance between encoder wheels changed to : ");
                            sendLog(dtoa((double) distance_between_encoder_wheels));
                            sendLog("\n");
                            /*funAngularAcc = (double)(value / COEF_SCALE_DISTANCE_BETWEEN_ENCODER_WHEELS);
                            sendLog("funAcc changed to : ");
                            sendLog(dtoa(funAngularAcc));
                            sendLog("\n");*/
                            break;
                    }
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="64bits">
                else if (type == VAR_64b) {
                    switch (var) {
                        case CODE_VAR_X_LD:
                        {
                            uint8_t *ptr = (uint8_t*) & x;
                            ptr[0] = RxDMABuffer[iArg3];
                            ptr[1] = RxDMABuffer[iArg4];
                            ptr[2] = RxDMABuffer[iArg5];
                            ptr[3] = RxDMABuffer[iArg6];
                            ptr[4] = RxDMABuffer[iArg7];
                            ptr[5] = RxDMABuffer[iArg8];
                            ptr[6] = RxDMABuffer[iArg9];
                            ptr[7] = RxDMABuffer[iArg10];
                            sendLongDouble(CODE_VAR_X_LD, x);
                            xc = x;
                            xf = x;
                            break;
                        }
                        case CODE_VAR_Y_LD:
                        {
                            uint8_t *ptr = (uint8_t*) & y;
                            ptr[0] = RxDMABuffer[iArg3];
                            ptr[1] = RxDMABuffer[iArg4];
                            ptr[2] = RxDMABuffer[iArg5];
                            ptr[3] = RxDMABuffer[iArg6];
                            ptr[4] = RxDMABuffer[iArg7];
                            ptr[5] = RxDMABuffer[iArg8];
                            ptr[6] = RxDMABuffer[iArg9];
                            ptr[7] = RxDMABuffer[iArg10];
                            sendLongDouble(CODE_VAR_Y_LD, y);
                            yc = y;
                            yf = y;
                            break;
                        }
                        case CODE_VAR_T_LD:
                        {
                            uint8_t *ptr = (uint8_t*) & theta;
                            ptr[0] = RxDMABuffer[iArg3];
                            ptr[1] = RxDMABuffer[iArg4];
                            ptr[2] = RxDMABuffer[iArg5];
                            ptr[3] = RxDMABuffer[iArg6];
                            ptr[4] = RxDMABuffer[iArg7];
                            ptr[5] = RxDMABuffer[iArg8];
                            ptr[6] = RxDMABuffer[iArg9];
                            ptr[7] = RxDMABuffer[iArg10];
                            sendLongDouble(CODE_VAR_X_LD, theta);
                            thetac = theta;
                            tf = theta;
                            break;
                        }
                        case CODE_VAR_COEF_DISSYMETRY_LD:
                        {
                            uint8_t *ptr = (uint8_t*) & coef_dissymmetry;
                            ptr[0] = RxDMABuffer[iArg3];
                            ptr[1] = RxDMABuffer[iArg4];
                            ptr[2] = RxDMABuffer[iArg5];
                            ptr[3] = RxDMABuffer[iArg6];
                            ptr[4] = RxDMABuffer[iArg7];
                            ptr[5] = RxDMABuffer[iArg8];
                            ptr[6] = RxDMABuffer[iArg9];
                            ptr[7] = RxDMABuffer[iArg10];
                            sendLongDouble(CODE_VAR_COEF_DISSYMETRY_LD, coef_dissymmetry);
                            break;
                        }
                        case CODE_VAR_MM_PER_TICKS_LD:
                        {
                            uint8_t *ptr = (uint8_t*) & mm_per_ticks;
                            ptr[0] = RxDMABuffer[iArg3];
                            ptr[1] = RxDMABuffer[iArg4];
                            ptr[2] = RxDMABuffer[iArg5];
                            ptr[3] = RxDMABuffer[iArg6];
                            ptr[4] = RxDMABuffer[iArg7];
                            ptr[5] = RxDMABuffer[iArg8];
                            ptr[6] = RxDMABuffer[iArg9];
                            ptr[7] = RxDMABuffer[iArg10];
                            sendLongDouble(CODE_VAR_MM_PER_TICKS_LD, mm_per_ticks);
                            break;
                        }
                        case CODE_VAR_RAD_PER_TICKS_LD:
                        {
                            uint8_t *ptr = (uint8_t*) & rad_per_ticks;
                            ptr[0] = RxDMABuffer[iArg3];
                            ptr[1] = RxDMABuffer[iArg4];
                            ptr[2] = RxDMABuffer[iArg5];
                            ptr[3] = RxDMABuffer[iArg6];
                            ptr[4] = RxDMABuffer[iArg7];
                            ptr[5] = RxDMABuffer[iArg8];
                            ptr[6] = RxDMABuffer[iArg9];
                            ptr[7] = RxDMABuffer[iArg10];
                            sendLongDouble(CODE_VAR_RAD_PER_TICKS_LD, rad_per_ticks);
                            break;
                        }
                    }
                    break;
                }// </editor-fold>
                // <editor-fold defaultstate="collapsed" desc="long double 64bits">
                else if (type == VAR_LD_64b) {
                    uint8_t *ptr = NULL;
                    uint8_t existing_var = 1;
                    switch (var) {
                        // <editor-fold defaultstate="collapsed" desc="position & setpoint">
                        case CODE_VAR_X_LD:
                            ptr = (uint8_t*) &x;
                            break;
                        case CODE_VAR_Y_LD:
                            ptr = (uint8_t*) &y;
                            break;
                        case CODE_VAR_T_LD:
                            ptr = (uint8_t*) &theta;
                            break;
                            
                        case CODE_VAR_XC_LD:
                            ptr = (uint8_t*) &xc;
                            break;
                        case CODE_VAR_YC_LD:
                            ptr = (uint8_t*) &yc;
                            break;
                        case CODE_VAR_TC_LD:
                            ptr = (uint8_t*) &thetac;
                            break;
                            
                        case CODE_VAR_XF_LD:
                            ptr = (uint8_t*) &xf;
                            break;
                        case CODE_VAR_YF_LD:
                            ptr = (uint8_t*) &yf;
                            break;
                        case CODE_VAR_TF_LD:
                            ptr = (uint8_t*) &tf;
                            break;
                            
                        case CODE_VAR_SPEED_X_LD:
                            ptr = (uint8_t*) &receivedSpeedX;
                            break;
                        case CODE_VAR_SPEED_Y_LD:
                            ptr = (uint8_t*) &receivedSpeedY;
                            break;
                        case CODE_VAR_SPEED_T_LD:
                            ptr = (uint8_t*) &receivedSpeedT;
                            break;// </editor-fold>
                        // <editor-fold defaultstate="collapsed" desc="Trajectory generation">
                        case CODE_VAR_TRAJ_LIN_SPEED_LD:
                            ptr = (uint8_t*) &linSpeed;
                            break;
                        case CODE_VAR_TRAJ_LIN_ACC_LD:
                            ptr = (uint8_t*) &linAcc;
                            break;
                        case CODE_VAR_TRAJ_ROT_SPEED_LD:
                            ptr = (uint8_t*) &rotSpeed;
                            break;
                        case CODE_VAR_TRAJ_ROT_ACC_LD:
                            ptr = (uint8_t*) &rotAcc;
                            break; // </editor-fold>
                        // <editor-fold defaultstate="collapsed" desc="Odometry">
                        case CODE_VAR_TICKS_PER_TURN_LD:
                            ptr = (uint8_t*) &ticksPerTurn;
                            break;
                        case CODE_VAR_WHEEL_DIAMETER_LD:
                            ptr = (uint8_t*) & wheelDiameter;
                            break;
                        case CODE_VAR_DISTANCE_CENTER_TO_WHEEL_LD:
                            ptr = (uint8_t*) &distanceCenterToWheel;
                            break;
                            
                        case CODE_VAR_WHEEL_DIAMETER0_LD:
                            ptr = (uint8_t*) &wheelDiameter0;
                            break;
                        case CODE_VAR_WHEEL_DIAMETER1_LD:
                            ptr = (uint8_t*) &wheelDiameter1;
                            break;
                        case CODE_VAR_WHEEL_DIAMETER2_LD:
                            ptr = (uint8_t*) &wheelDiameter2;
                            break; // </editor-fold>
                        // <editor-fold defaultstate="collapsed" desc="PID">
                        case CODE_VAR_P_SPEED_0_LD:
                            ptr = (uint8_t*) &pidSpeed0.Kp;
                            break;
                        case CODE_VAR_I_SPEED_0_LD:
                            ptr = (uint8_t*) &pidSpeed0.Ki;
                            break;
                        case CODE_VAR_D_SPEED_0_LD:
                            ptr = (uint8_t*) &pidSpeed0.Kd;
                            break;
                        case CODE_VAR_P_SPEED_1_LD:
                            ptr = (uint8_t*) &pidSpeed1.Kp;
                            break;
                        case CODE_VAR_I_SPEED_1_LD:
                            ptr = (uint8_t*) &pidSpeed1.Ki;
                            break;
                        case CODE_VAR_D_SPEED_1_LD:
                            ptr = (uint8_t*) &pidSpeed1.Kd;
                            break;
                        case CODE_VAR_P_SPEED_2_LD:
                            ptr = (uint8_t*) &pidSpeed2.Kp;
                            break;
                        case CODE_VAR_I_SPEED_2_LD:
                            ptr = (uint8_t*) &pidSpeed2.Ki;
                            break;
                        case CODE_VAR_D_SPEED_2_LD:
                            ptr = (uint8_t*) &pidSpeed2.Kd;
                            break;
                        case CODE_VAR_P_DISTANCE_LD:
                            ptr = (uint8_t*) &pidDistance.Kp;
                            break;
                        case CODE_VAR_I_DISTANCE_LD:
                            ptr = (uint8_t*) &pidDistance.Ki;
                            break;
                        case CODE_VAR_D_DISTANCE_LD:
                            ptr = (uint8_t*) &pidDistance.Kd;
                            break;
                        case CODE_VAR_P_ANGLE_LD:
                            ptr = (uint8_t*) &pidAngle.Kp;
                            break;
                        case CODE_VAR_I_ANGLE_LD:
                            ptr = (uint8_t*) &pidAngle.Ki;
                            break;
                        case CODE_VAR_D_ANGLE_LD:
                            ptr = (uint8_t*) &pidAngle.Kd;
                            break; // </editor-fold>
                        default:
                            existing_var = 0;
                            break;
                    }
                    if (existing_var) {
                        ptr[0] = RxDMABuffer[iArg3];
                        ptr[1] = RxDMABuffer[iArg4];
                        ptr[2] = RxDMABuffer[iArg5];
                        ptr[3] = RxDMABuffer[iArg6];
                        ptr[4] = RxDMABuffer[iArg7];
                        ptr[5] = RxDMABuffer[iArg8];
                        ptr[6] = RxDMABuffer[iArg9];
                        ptr[7] = RxDMABuffer[iArg10];
                    }
                    break;
                }// </editor-fold>

            } // </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Get">
            case RX_CODE_GET:
            {
                if (size != RX_SIZE_GET)
                    return;

                unsigned char varCode = RxDMABuffer[iArg1];

                switch (varCode) {
                    case VAR_STATE:
                        //sendLog(itoa(state));
                        break;
                    case CODE_VAR_ARRIVED:
                        sendVar8(CODE_VAR_ARRIVED,arrived);
                        //sendLog("get_arrived\n");
                        break;
                    case CODE_VAR_BAT:{
                        double vbat = VBAT;
                        sendDouble(CODE_VAR_BAT, &vbat);
                        break;
                    }
                    case CODE_VAR_ALLPID:
                        //sendAllPID();
                        sendAllPIDLongDouble();
                        break;
                    case CODE_VAR_ODO:
                        sendLongDouble(CODE_VAR_COEF_DISSYMETRY_LD,coef_dissymmetry);
                        sendLongDouble(CODE_VAR_MM_PER_TICKS_LD,mm_per_ticks);
                        sendLongDouble(CODE_VAR_RAD_PER_TICKS_LD,rad_per_ticks);
                        break;
                    default:
                        break;
                }
                break;
            }// </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Servo">
            case RX_CODE_SERVO:
            {
                //sendLog("servo code detected\n");
                if (size != RX_SIZE_SERVO)
                    return;

                unsigned char id = RxDMABuffer[iArg1];
                uint16_t val = (RxDMABuffer[iArg2] << 8) + RxDMABuffer[iArg3];
                /*sendLog("id = ");
                sendLog(itoa((int)id));
                sendLog(" & val =");
                sendLog(itoa(val));
                sendLog("\n");*/
                
                
                if (id > ID_MAX_SERVO)
                    return;
                if (val > 20000)
                    return;

                //servoUs(id, (double) val);
                //cout << "servo " << (int)id << " " << val << endl;
                sendLog("servo ");
                sendLog(itoa((int)id));
                sendLog(" ");
                sendLog(itoa(val));
                sendLog("\n");
                break;
            }// </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Motor">
            case RX_CODE_MOTOR:
            {
                if (size != RX_SIZE_MOTOR)
                    return;

                unsigned char id = RxDMABuffer[iArg1];
                char val = RxDMABuffer[iArg2];
                if (id > 2) //holonome => motors e[0;1;2]
                    return;
                if (val < -100 || val > 100)
                    return;

                motor(id,val);
                sendLog("motor ");
                sendLog(itoa((int)id));
                sendLog(" ");
                sendLog(itoa((int)val));
                sendLog("\n");
                //cout << "motor " << (int)id << " " << (int)val << endl;
                break;
            }// </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="AX12">
            case RX_CODE_AX12:
            {
                if (size != RX_SIZE_AX12)
                    return;

                unsigned char id = RxDMABuffer[iArg1];
                unsigned char H = RxDMABuffer[iArg2];
                unsigned char L = RxDMABuffer[iArg3];

                if (id != 1 && id != 3)
                    return;
                if (H > 3)
                    return;

                switch (id) {
                    case 1:
                        rotateToAX12(AX12_ID_1, L, H);
                        sendLog("AX12 1 ");
                        sendLog(itoa((int)H));
                        sendLog(" ");
                        sendLog(itoa((int)L));
                        sendLog("\n");
                        //cout << "AX12 1 " << (int)H << " " << (int)L << endl;
                        break;
                    case 3:
                        rotateToAX12(AX12_ID_3, L, H);
                        sendLog("AX12 3 ");
                        sendLog(itoa((int)H));
                        sendLog(" ");
                        sendLog(itoa((int)L));
                        sendLog("\n");
                        //cout << "AX12 3 " << (int)H << " " << (int)L << endl;
                        break;
                    default:
                        //sendLog("error invalid AX12 id (1 or 3)\n");
                        break;
                }
                break;
            }// </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Go">
            case RX_CODE_GO:
            {
                if (size != RX_SIZE_GO)
                    return;

                unsigned char option = RxDMABuffer[iArg1];
                int16_t x_arg = (RxDMABuffer[iArg2] << 8) + RxDMABuffer[iArg3];
                int16_t y_arg = (RxDMABuffer[iArg4] << 8) + RxDMABuffer[iArg5];
                if (option & MASK_OPTION_RELATIVE) {
                    x_arg += x;
                    y_arg += y;
                }
                if (x_arg >= 0 && x_arg <= 2000 && y_arg >= 0 && y_arg <= 3000) {
                    receivedX = (double) x_arg;
                    receivedY = (double) y_arg;
                    newPosReceived = 1;
                    if (option & MASK_OPTION_REVERSE) {
                        newPosBackReceived = 1;
                    }
                    //cout << "go " << x_arg << " " << y_arg << endl;
                    sendLog("go ");
                    sendLog(itoa(x_arg));
                    sendLog(" ");
                    sendLog(itoa(y_arg));
                    sendLog("\n");
                    /*sendLog("received [go] : x = ");
                    sendLog(itoa(x));
                    sendLog(" & y = ");
                    sendLog(itoa(y));
                    sendLog("\n");*/
                }
                break;
            }// </editor-fold>

                // <editor-fold defaultstate="collapsed" desc="Turn">
            case RX_CODE_TURN:
            {
                if (size != RX_SIZE_TURN)
                    return;

                unsigned char option = RxDMABuffer[iArg1];
                int16_t t = (RxDMABuffer[iArg2] << 8) + RxDMABuffer[iArg3];
                //double test = (inBuf[start + 3] << 8) + inBuf[start + 4]; // m�me r�sultat ? � v�rifier
                double t_d = (double) t;
                t_d = t_d * 2 * PI / 360;
                if (option & MASK_OPTION_RELATIVE)
                    t_d += theta;

                receivedTheta = t_d;
                newAngleReceived = 1;
                sendLog("turn ");
                sendLog(itoa(t));
                sendLog("\n");
                //cout << "turn " << t_d << endl;
                break;
            }// </editor-fold>

            default:
                break;
        }
    }
}

void sendPos(){
    uint16_t intX = (uint16_t)x;
    uint16_t intY = (uint16_t)y;
    int16_t intT = (int16_t)((theta*360)/(2*PI));
    uint8_t i;
    
    uint8_t buffer[TX_SIZE_VAR_16B + 1];
    buffer[0] = TX_SIZE_VAR_16B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = CODE_VAR_X;
    buffer[3] = (uint8_t)(intX >> 8);
    buffer[4] = (uint8_t)(intX & 0xFF);
    buffer[5] = 0;
    for(i = 0; i < TX_SIZE_VAR_16B; i++){
        buffer[5] += buffer[i];	//checksum
    }
    send(buffer,TX_SIZE_VAR_16B + 1);
    
    buffer[0] = TX_SIZE_VAR_16B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = CODE_VAR_Y;
    buffer[3] = (uint8_t)(intY >> 8);
    buffer[4] = (uint8_t)(intY & 0xFF);
    buffer[5] = 0;
    for(i = 0; i < TX_SIZE_VAR_16B; i++){
        buffer[5] += buffer[i];	//checksum
    }
    send(buffer,TX_SIZE_VAR_16B + 1);
    
    buffer[0] = TX_SIZE_VAR_16B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = CODE_VAR_T;
    buffer[3] = (uint8_t)(intT >> 8);
    buffer[4] = (uint8_t)(intT & 0xFF);
    buffer[5] = 0;
    for(i = 0; i < TX_SIZE_VAR_16B; i++){
        buffer[5] += buffer[i];	//checksum
    }
    send(buffer,TX_SIZE_VAR_16B + 1);
}
void sendRupt(){
    uint8_t ruptBitField1 = 0;
    uint8_t ruptBitField2 = 0;
    ruptBitField1 += RUPT_ASS_0;
    ruptBitField1 += RUPT_ASS_1 << 1;
    ruptBitField1 += RUPT_ASS_2 << 2;
    ruptBitField1 += RUPT_ASS_3 << 3;
    
    ruptBitField1 += RUPT_ACT_0 << 4;
    ruptBitField1 += RUPT_ACT_1 << 5;
    ruptBitField1 += RUPT_ACT_2 << 6;
    ruptBitField1 += RUPT_ACT_3 << 7;
    ruptBitField2 += RUPT_ACT_4;
    ruptBitField2 += RUPT_ACT_5 << 1;
    
    uint8_t i;
    
    uint8_t buffer[TX_SIZE_VAR_16B + 1];
    buffer[0] = TX_SIZE_VAR_16B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = CODE_VAR_RUPT;
    buffer[3] = ruptBitField1;
    buffer[4] = ruptBitField2;
    buffer[5] = 0;
    for(i = 0; i < TX_SIZE_VAR_16B; i++){
        buffer[5] += buffer[i];	//checksum
    }
    send(buffer,TX_SIZE_VAR_16B + 1);
}
void sendUS(){
    uint8_t i,j;
    uint8_t buffer[TX_SIZE_VAR_16B + 1];
    buffer[0] = TX_SIZE_VAR_16B;
    buffer[1] = TX_CODE_VAR;
    for(i = 0; i < NB_US; i++){
        uint16_t dist = (uint16_t)(US[i] * 30 / 1400);
        buffer[2] = CODE_VAR_US + i;
        buffer[3] = (uint8_t)(dist >> 8);
        buffer[4] = (uint8_t)(dist & 0xFF);
        buffer[5] = 0;
        for(j = 0; j < TX_SIZE_VAR_16B; j++){
            buffer[5] += buffer[j];	//checksum
        }
        send(buffer,TX_SIZE_VAR_16B + 1);
    }   
}
void send(uint8_t *str,uint16_t size){
    if(verbose == 0)
        return;
    IEC1bits.U2TXIE = 0;    //disable Tx interrupt
    uint16_t i = 0;
    //uint8_t saveTxOn2 = TxOn2;  //TxOn2 could change during an interrupt
    if(TxOn2 == 1)
        push2(str[0]);
    for(i = 1; i < size; i++){
        push2(str[i]);
    }
    if(TxOn2 == 0){
        TxOn2 = 1;
        U2TXREG = str[0];
    }
    IEC1bits.U2TXIE = 1;    //reenable TX interrupt
}
void sendLog(char *str){
    if(verbose == 0)
        return;
    int i;
    int len = strlen(str);
    uint8_t header[2];
    header[0] = 2 + len;
    header[1] = TX_CODE_LOG;
    uint8_t checksum = header[0] + header[1];
    for(i = 0; i < len; i++){
        checksum += str[i];
    }   
    send(header,2);
    send((uint8_t*)str,len);
    send(&checksum,1);
}
void plot(uint8_t id,uint32_t value){
    uint8_t i;
    uint32_t time = micros();
    
    uint8_t buffer[TX_SIZE_PLOT + 1];
    buffer[0] = TX_SIZE_PLOT;
    buffer[1] = TX_CODE_PLOT;
    buffer[2] = id;
    buffer[3] = time >> 24;
    buffer[4] = time >> 16;
    buffer[5] = time >> 8;
    buffer[6] = time & 0xFF;
    buffer[7] = value >> 24;
    buffer[8] = value >> 16;
    buffer[9] = value >> 8;
    buffer[10] = value & 0xFF;
    buffer[11] = 0;
    for(i = 0; i < TX_SIZE_PLOT; i++){
        buffer[11] += buffer[i];	//checksum
    }
    send(buffer,TX_SIZE_PLOT + 1);
}
void sendAllPID(){
    sendVar32(CODE_VAR_P_SPEED_L,(uint32_t)(pidSpeed0.Kp*COEF_SCALE_PID));
    sendVar32(CODE_VAR_I_SPEED_L,(uint32_t)(pidSpeed0.Ki*COEF_SCALE_PID));
    sendVar32(CODE_VAR_D_SPEED_L,(uint32_t)(pidSpeed0.Kd*COEF_SCALE_PID));
    
    sendVar32(CODE_VAR_P_SPEED_R,(uint32_t)(pidSpeed1.Kp*COEF_SCALE_PID));
    sendVar32(CODE_VAR_I_SPEED_R,(uint32_t)(pidSpeed0.Ki*COEF_SCALE_PID));
    sendVar32(CODE_VAR_D_SPEED_R,(uint32_t)(pidSpeed0.Kd*COEF_SCALE_PID));
    
    sendVar32(CODE_VAR_P_DISTANCE,(uint32_t)(pidSpeed2.Kp*COEF_SCALE_PID));
    sendVar32(CODE_VAR_I_DISTANCE,(uint32_t)(pidSpeed2.Ki*COEF_SCALE_PID));
    sendVar32(CODE_VAR_D_DISTANCE,(uint32_t)(pidSpeed2.Kd*COEF_SCALE_PID));
    
    sendVar32(CODE_VAR_P_ANGLE,(uint32_t)(pidAngle.Kp*COEF_SCALE_PID));
    sendVar32(CODE_VAR_I_ANGLE,(uint32_t)(pidAngle.Ki*COEF_SCALE_PID));
    sendVar32(CODE_VAR_D_ANGLE,(uint32_t)(pidAngle.Kd*COEF_SCALE_PID));
}
void sendAllPIDLongDouble(){
    sendLongDouble(CODE_VAR_P_SPEED_0_LD,pidSpeed0.Kp);
    sendLongDouble(CODE_VAR_I_SPEED_0_LD,pidSpeed0.Ki);
    sendLongDouble(CODE_VAR_D_SPEED_0_LD,pidSpeed0.Kd);
    
    sendLongDouble(CODE_VAR_P_SPEED_1_LD,pidSpeed1.Kp);
    sendLongDouble(CODE_VAR_I_SPEED_1_LD,pidSpeed1.Ki);
    sendLongDouble(CODE_VAR_D_SPEED_1_LD,pidSpeed1.Kd);
    
    sendLongDouble(CODE_VAR_P_SPEED_2_LD,pidSpeed2.Kp);
    sendLongDouble(CODE_VAR_I_SPEED_2_LD,pidSpeed2.Ki);
    sendLongDouble(CODE_VAR_D_SPEED_2_LD,pidSpeed2.Kd);
    
    sendLongDouble(CODE_VAR_P_DISTANCE_LD,pidDistance.Kp);
    sendLongDouble(CODE_VAR_I_DISTANCE_LD,pidDistance.Ki);
    sendLongDouble(CODE_VAR_D_DISTANCE_LD,pidDistance.Kd);
    
    sendLongDouble(CODE_VAR_P_ANGLE_LD,pidAngle.Kp);
    sendLongDouble(CODE_VAR_I_ANGLE_LD,pidAngle.Ki);
    sendLongDouble(CODE_VAR_D_ANGLE_LD,pidAngle.Kd);
}
void sendVar8(uint8_t varCode, uint8_t var){
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_8B + 1];
    buffer[0] = TX_SIZE_VAR_8B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = var;
    buffer[4] = 0;
    for(i = 0; i < TX_SIZE_VAR_8B; i++){
        buffer[4] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_8B + 1);
}
void sendVar32(uint8_t varCode, uint32_t var){
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_32B + 1];
    buffer[0] = TX_SIZE_VAR_32B;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = (uint8_t)(var >> 24);
    buffer[4] = (uint8_t)(var >> 16);
    buffer[5] = (uint8_t)(var >> 8);
    buffer[6] = (uint8_t)(var & 0xFF);
    buffer[7] = 0;
    for(i = 0; i < TX_SIZE_VAR_32B; i++){
        buffer[7] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_32B + 1);
}
void sendDouble(uint8_t varCode, double *ptrVar){
    uint8_t *ptrChar = (uint8_t*)ptrVar;
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_DOUBLE + 1];
    buffer[0] = TX_SIZE_VAR_DOUBLE;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = ptrChar[0];
    buffer[4] = ptrChar[1];
    buffer[5] = ptrChar[2];
    buffer[6] = ptrChar[3];
    buffer[7] = 0;
    for(i = 0; i < TX_SIZE_VAR_DOUBLE; i++){
        buffer[7] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_DOUBLE + 1);
}
void sendLongDouble(uint8_t varCode, long double var){
    long double *ptrVar = &var;
    uint8_t *ptrChar = (uint8_t*)ptrVar;
    uint8_t  i;
	uint8_t buffer[TX_SIZE_VAR_LONG_DOUBLE + 1];
    buffer[0] = TX_SIZE_VAR_LONG_DOUBLE;
    buffer[1] = TX_CODE_VAR;
    buffer[2] = varCode;
    buffer[3] = ptrChar[0];
    buffer[4] = ptrChar[1];
    buffer[5] = ptrChar[2];
    buffer[6] = ptrChar[3];
    buffer[7] = ptrChar[4];
    buffer[8] = ptrChar[5];
    buffer[9] = ptrChar[6];
    buffer[10] = ptrChar[7];
    buffer[11] = 0;
    for(i = 0; i < TX_SIZE_VAR_LONG_DOUBLE; i++){
        buffer[11] += buffer[i];	//checksum
    }
	send(buffer,TX_SIZE_VAR_LONG_DOUBLE + 1);
}
void sendPosLongDouble(){
    sendLongDouble(CODE_VAR_X_LD,x);
    sendLongDouble(CODE_VAR_Y_LD,y);
    sendLongDouble(CODE_VAR_T_LD,theta);
}
