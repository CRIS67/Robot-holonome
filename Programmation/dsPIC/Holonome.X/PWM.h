/* 
 * File: PWM.h  
 * Author: Quentin BOYER
 * Comments: 
 * Revision history: 1
 */

#ifndef PWM_H
#define	PWM_H

#include <xc.h> // include processor files - each processor file is guarded. 
#include <stdint.h>
#include "timer.h"
#include "constant.h"
void initPWM();
void initPWM1();
void initPWM2();
void initPWM3();
void initPWM4();
void initPWM5();
void initPWM6();

void motor(uint8_t id, double value);
void sendMotor0(double value);
void sendMotor1(double value);
void sendMotor2(double value);
void sendMotor(double val0, double val1, double val2);

#endif	/* PWM_H */