#ifndef CONSTANT_H
#define	CONSTANT_H

#define SECURITE_DISTANCE   120
#define SECURITE_ANGLE      2
#define SECURITE_SPEED      40

// <editor-fold defaultstate="collapsed" desc="PID">
//PID speed left    units : rad/s -> V
#define KP_SPEED_LEFT               0.1//1.5//1//0.38//0.57//0.576
#define KI_SPEED_LEFT               0.35//2//0//0.00535//0.05
#define KD_SPEED_LEFT               0//0.5//0//0.001675//0.0013375//0.005
#define BIAS_SPEED_LEFT             0
#define T_SPEED_LEFT                0.01    //s
#define SMOOTHING_FACTOR_SPEED_LEFT 0.8//0.03
#define SATURATION_SPEED_LEFT       VSAT    //unit : voltage

//PID speed right   units : rad/s -> V
#define KP_SPEED_RIGHT              0.05//1.5//1//0.38//0.57//0.576
#define KI_SPEED_RIGHT              0.35//2//0//0.00535//0.05
#define KD_SPEED_RIGHT              0//0.5//0//0.001675//0.0013375//0.005
#define BIAS_SPEED_RIGHT            0
#define T_SPEED_RIGHT               0.01
#define SMOOTHING_FACTOR_SPEED_RIGHT 0.8//0.03
#define SATURATION_SPEED_RIGHT      VSAT    //unit : voltage

//PID distance      units : mm -> rad/s
#define KP_DISTANCE                 0//0.015//0.15//0.04
#define KI_DISTANCE                 0
#define KD_DISTANCE                 0//0.001//0//0.0065
#define BIAS_DISTANCE               0
#define T_DISTANCE                  0.01
#define SMOOTHING_FACTOR_DISTANCE   1
#define SATURATION_DISTANCE         1000000    //unit : mm/s

//PID angle         units : rad -> rad/s
#define KP_ANGLE                    0.002//1000//20//80//15//80//120//90//30//60
#define KI_ANGLE                    0
#define KD_ANGLE                    0//1//2//1//0//6.5
#define BIAS_ANGLE                  0
#define T_ANGLE                     0.01
#define SMOOTHING_FACTOR_ANGLE      1
#define SATURATION_ANGLE            1000000    //unit : mm/s

//#define MAX_I    100

#define KP_SPEED                    3
#define KI_SPEED                    10
#define KD_SPEED                    0

//PID speed 0    units : tr/s -> V
#define KP_SPEED_0                  KP_SPEED    //10//1.5//1//0.38//0.57//0.576
#define KI_SPEED_0                  KI_SPEED    //50//2//0//0.00535//0.05
#define KD_SPEED_0                  KD_SPEED    //0//0.5//0//0.001675//0.0013375//0.005
#define BIAS_SPEED_0                0
#define T_SPEED_0                   0.02    //s
#define SMOOTHING_FACTOR_SPEED_0    0.03
#define SATURATION_SPEED_0          VSAT    //unit : voltage

//PID speed 1    units : tr/s -> V
#define KP_SPEED_1                  KP_SPEED    //10//1.5//1//0.38//0.57//0.576
#define KI_SPEED_1                  KI_SPEED    //50//2//0//0.00535//0.05
#define KD_SPEED_1                  KD_SPEED    //0//0.5//0//0.001675//0.0013375//0.005
#define BIAS_SPEED_1                0
#define T_SPEED_1                   0.02
#define SMOOTHING_FACTOR_SPEED_1    0.03
#define SATURATION_SPEED_1          VSAT    //unit : voltage

//PID speed 2    units : tr/s -> V
#define KP_SPEED_2                  KP_SPEED    //10//1.5//1//0.38//0.57//0.576
#define KI_SPEED_2                  KI_SPEED    //50//2//0//0.00535//0.05
#define KD_SPEED_2                  KD_SPEED    //0//0.5//0//0.001675//0.0013375//0.005
#define BIAS_SPEED_2                0
#define T_SPEED_2                   0.02
#define SMOOTHING_FACTOR_SPEED_2    0.03
#define SATURATION_SPEED_2          VSAT    //unit : voltage



// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Clock">
#define CLOCK_M     35
#define CLOCK_N1    2
#define CLOCK_N2    2
#define CLOCK_SOURCE_FREQ_HZ   16000000

//#define CLOCK_FREQ  CLOCK_SOURCE_FREQ * CLOCK_M / (CLOCK_N1 * CLOCK_N2)
#define CLOCK_FREQ_HZ   140000000
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="UART">
#define BAUDRATE 9600
#define BRGVAL 455  //((CLOCK_FREQ_HZ/2/BAUDRATE)/16) - 1   9600
#define BRGVAL2 34//7 //34//6//34//37   //  500 000
//#define BRGVAL2 37//7 //34//6//34//37   //  500 000
#define UART2_HIGH_SPEED 1  //coef *4 : BRGH

#define TX_SIZE 1000     //size of Tx buffer
#define RX_SIZE 100     //size of Rx buffer
#define RX_DMA_SIZE 1000 //

#define COEF_SCALE_PID  1000000
#define COEF_SCALE_COEF_DISSYMETRY                      10000
#define COEF_SCALE_MM_PER_TICKS                         1000000
#define COEF_SCALE_DISTANCE_BETWEEN_ENCODER_WHEELS      1000

#define RX_CODE_START   1
#define RX_CODE_STOP    2
#define RX_CODE_SET     3
#define RX_CODE_GET     4
#define RX_CODE_SERVO   5
#define RX_CODE_MOTOR   6
#define RX_CODE_AX12    7
#define RX_CODE_GO      8
#define RX_CODE_TURN    9

#define RX_SIZE_START   2
#define RX_SIZE_STOP    2
#define RX_SIZE_SET       // var,type,value
#define RX_SIZE_GET     3 // var
#define RX_SIZE_SERVO   5 // id,value_H,value_L
#define RX_SIZE_MOTOR   4 // id,value
#define RX_SIZE_AX12    5 // id,value_H,value_L
#define RX_SIZE_GO      7 // option,x_H,x_L,y_H,y_L
#define RX_SIZE_TURN    5 // option,t_H,t_L

#define TX_CODE_VAR     1
#define TX_CODE_LOG     2
#define TX_CODE_PLOT    3

#define TX_SIZE_VAR      //�a d�pend
#define TX_SIZE_VAR_8B  4
#define TX_SIZE_VAR_16B 5
#define TX_SIZE_VAR_32B 7
#define TX_SIZE_VAR_DOUBLE 7
#define TX_SIZE_VAR_LONG_DOUBLE 11

#define TX_SIZE_PLOT    11   

#define MASK_OPTION_RELATIVE    0x2
#define MASK_OPTION_REVERSE     0x1

#define VAR_STATE   0
#define VAR_BAT     1
#define VAR_P_SPEED 2
#define VAR_I_SPEED 3
#define VAR_D_SPEED 4


#define CODE_VAR_X          1
#define CODE_VAR_Y          2
#define CODE_VAR_T          3

/*#define CODE_VAR_X_LD       6
#define CODE_VAR_Y_LD       7
#define CODE_VAR_T_LD       8*/

#define CODE_VAR_RUPT       4
#define CODE_VAR_VERBOSE    5

#define CODE_VAR_ARRIVED    6

#define CODE_VAR_ALLPID     9

#define CODE_VAR_STATE       10
#define CODE_VAR_BAT         11

#define CODE_VAR_P_SPEED_L   12
#define CODE_VAR_I_SPEED_L   13
#define CODE_VAR_D_SPEED_L   14
#define CODE_VAR_P_SPEED_R   15
#define CODE_VAR_I_SPEED_R   16
#define CODE_VAR_D_SPEED_R   17
#define CODE_VAR_P_DISTANCE  18
#define CODE_VAR_I_DISTANCE  19
#define CODE_VAR_D_DISTANCE  20
#define CODE_VAR_P_ANGLE     21
#define CODE_VAR_I_ANGLE     22
#define CODE_VAR_D_ANGLE     23

#define CODE_VAR_ODO         29

#define CODE_VAR_COEF_DISSYMETRY                    30
#define CODE_VAR_MM_PER_TICKS                       31
#define CODE_VAR_DISTANCE_BETWEEN_ENCODER_WHEELS    32

#define CODE_VAR_COEF_DISSYMETRY_LD                 33
#define CODE_VAR_MM_PER_TICKS_LD                    34
#define CODE_VAR_RAD_PER_TICKS_LD                   35

#define CODE_VAR_P_SPEED_0_LD   42
#define CODE_VAR_I_SPEED_0_LD   43
#define CODE_VAR_D_SPEED_0_LD   44
#define CODE_VAR_P_SPEED_1_LD   45
#define CODE_VAR_I_SPEED_1_LD   46
#define CODE_VAR_D_SPEED_1_LD   47
#define CODE_VAR_P_SPEED_2_LD   48
#define CODE_VAR_I_SPEED_2_LD   49
#define CODE_VAR_D_SPEED_2_LD   50
#define CODE_VAR_P_DISTANCE_LD  51
#define CODE_VAR_I_DISTANCE_LD  52
#define CODE_VAR_D_DISTANCE_LD  53
#define CODE_VAR_P_ANGLE_LD     54
#define CODE_VAR_I_ANGLE_LD     55
#define CODE_VAR_D_ANGLE_LD     56

#define CODE_VAR_TRAJ_LIN_SPEED_LD  60
#define CODE_VAR_TRAJ_LIN_ACC_LD    61

#define CODE_VAR_TRAJ_ROT_SPEED_LD  62
#define CODE_VAR_TRAJ_ROT_ACC_LD    63

#define CODE_VAR_DISTANCE_MAX_LD    64

#define CODE_VAR_TICKS_PER_TURN_LD              70
#define CODE_VAR_WHEEL_DIAMETER_LD              71
#define CODE_VAR_DISTANCE_CENTER_TO_WHEEL_LD    72

#define CODE_VAR_WHEEL_DIAMETER0_LD             73
#define CODE_VAR_WHEEL_DIAMETER1_LD             74
#define CODE_VAR_WHEEL_DIAMETER2_LD             75

#define CODE_VAR_X_LD    80
#define CODE_VAR_Y_LD    81
#define CODE_VAR_T_LD    82

#define CODE_VAR_XC_LD    83
#define CODE_VAR_YC_LD    84
#define CODE_VAR_TC_LD    85

#define CODE_VAR_XF_LD    86
#define CODE_VAR_YF_LD    87
#define CODE_VAR_TF_LD    88

#define CODE_VAR_MODE_ASSERV    110

#define CODE_VAR_SPEED_X_LD     111
#define CODE_VAR_SPEED_Y_LD     112
#define CODE_VAR_SPEED_T_LD     113

/*
extern volatile long double ticksPerTurn;
extern volatile long double wheelDiameter;
extern volatile long double distanceCenterToWheel;*/

#define CODE_VAR_US     100 //attention range [100 ; 100 + NB_US - 1]

#define VAR_8b      0
#define VAR_16b     1
#define VAR_32b     2
#define VAR_64b     3
#define VAR_LD_64b  4


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Motor control">
#define FORWARD     1
#define BACKWARD    0

#define PWM_PR_L    PHASE3
#define PWM_PR_R    SPHASE3

#define PWM_PR_0    PHASE3      //MOTOR0 -> PWM3H
#define PWM_PR_1    SPHASE3     //MOTOR1 -> PWM3L
#define PWM_PR_2    SPHASE2     //MOTOR2 -> PWM2L

#define PWM_0       PDC3            //PWM3H pin93  RB10 PWM_ASS_0
#define PWM_1       SDC3            //PWM3L pin94  RB11 PWM_ASS_1
#define PWM_2       SDC2            //PWM2L pin99  RB13 PWM_ASS_2

#define SENS_0      LATGbits.LATG0  //      pin90 RG0  SENS_ASS_0
#define SENS_1      LATGbits.LATG1  //      pin89 RG1  SENS_ASS_1
#define SENS_2      LATGbits.LATG12 //      pin96 RG12 SENS_ASS_2

#define VBAT        (((double)ADC1BUF0*5.7*3.3)/1024)//12  //� remplacer plus tard par lecture de la tension ?

#define VSAT        8  //saturation pour brider la vitesse  2 -> 6V au multim�tre 

#define CORRECTEUR_BO_0 1.37
#define CORRECTEUR_BO_1 1.16
#define CORRECTEUR_BO_2 1.24

#define THRESHOLD_MOTOR 0.8

#define PERCENTAGE_DEADBAND 0
#define DEAD_ZONE   1.3   //tension min qui fait tourner le moteur A MESURER
#define COEF_MOT_BO 0.65//0.428571    //  250*12/7000
//#define VMIN        1//0.3   //arreter les moteurs si la commande trop faible

//#define ACC_MAX 0.5
#define ACC_MAX 0.1

#define MAX_ERROR_D     8//1//10      //mm
#define MAX_ERROR_A     0.05//0.02//0.01//0.01rad ~= 0.57�

#define MAX_ERROR_D_INF     5//1//10      //mm
#define MAX_ERROR_D_SUP     10//1//10      //mm
#define MAX_ERROR_A_INF     0.05//0.02//0.01//0.01rad ~= 0.57�
#define MAX_ERROR_A_SUP     0.1//0.02//0.01//0.01rad ~= 1.04�

#define MAX_SPEED_STOP  5 // ENCODER_WHEEL_RADIUS    //rad/s -> 5mm/s

#define N_ASSERV        20  //nombre d'it�rations entre 2 boucles d'asserv'
#define DIST_AIM_POINT  50  //distance au dessus de laquelle le robot vise le point final


#define MODE_POSITION   0
#define MODE_SPEED      1
//#define SMOOTHING_FACTOR    0.2
//#define N_SMOOTHING     5

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I/O">

#define NB_US       6               //number of sensors
#define N_US        20              //loop iterations

#define PWM_R       PDC3            //PWM5L pin7  RD2 PWM_ASS_1
#define PWM_L       SDC3            //PWM5H pin6  RD1 PWM_ASS_0
#define SENS_L      LATGbits.LATG1  //      pin18 RE8 SENS_ASS_0
#define SENS_R      LATGbits.LATG0  //      pin19 RE9 SENS_ASS_1

#define RUPT_ASS_0  PORTEbits.RE8
#define RUPT_ASS_1  PORTGbits.RG10
#define RUPT_ASS_2  PORTAbits.RA12
#define RUPT_ASS_3  PORTEbits.RE9

#define RUPT_ACT_0  PORTFbits.RF9
#define RUPT_ACT_1  PORTBbits.RB1
#define RUPT_ACT_2  PORTBbits.RB0
#define RUPT_ACT_3  PORTAbits.RA1
#define RUPT_ACT_4  PORTAbits.RA0
#define RUPT_ACT_5  PORTAbits.RA11

#define SIGNAL_JUMPER   PORTFbits.RF10

#define ECHO_US_0   PORTAbits.RA8
#define ECHO_US_1   PORTDbits.RD14
#define ECHO_US_2   PORTEbits.RE14
#define ECHO_US_3   PORTEbits.RE12
#define ECHO_US_4   PORTFbits.RF13
#define ECHO_US_5   PORTCbits.RC11

#define TRIG_US_0   LATBbits.LATB4
#define TRIG_US_1   LATDbits.LATD15
#define TRIG_US_2   LATEbits.LATE15
#define TRIG_US_3   LATEbits.LATE13
#define TRIG_US_4   LATFbits.LATF12
#define TRIG_US_5   LATGbits.LATG11

#define US_NUM_0    LATCbits.LATC1
#define US_NUM_1    LATCbits.LATC0


#define LED_0       LATGbits.LATG3
#define LED_1       LATFbits.LATF4
#define LED_2       LATFbits.LATF5

#define SENS_ASS_0  LATGbits.LATG0
#define SENS_ASS_1  LATGbits.LATG1
#define SENS_ACT_0  LATGbits.LATG13
#define SENS_ACT_1  LATGbits.LATG12

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Odometry">
#define PI                                  3.14159265358979323846
#define ENCODER_WHEEL_RADIUS                24.6                    //mm         
#define DISTANCE_BETWEEN_ENCODER_WHEELS     293.83//300//295.8918449447346863    //mm

#define COEF_DISSYMETRY                     1.010
#define ENCODER_WHEEL_DIAMETER              43.56
#define TICKS_PER_TURN                      1496.88


#define MM_PER_TICKS                        PI * ENCODER_WHEEL_DIAMETER / TICKS_PER_TURN
#define RAD_PER_TICKS                       0.0001405

#define ANGLE0  PI
#define ANGLE1  (5.0*PI)/3.0
#define ANGLE2  (7.0*PI)/3.0

#define WHEEL_DIAMETER  63.8
#define DISTANCE_CENTER_TO_WHEEL  124.4//115.782
//#define TICKS_PER_RAD                       15572.957676416326850467129424503
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ADC">
#define AUTO_SAMPLE 1   //0-Sampling begins when SAMP bit is set / 1-Sampling begins immediately after last conversion; SAMP bit is auto-set
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Path generation">
#define ROTATION_SPEED      0.002   //rad
#define LINEAR_SPEED        0.1     //mm
#define DELAY_SPEED         1       //ms

#define ACCELERATION_MAX    3500     //mm.s^-2
#define SPEED_MAX           1000000    //mm.s^-1
#define TE                  0.01    //s
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Servomotors">
#define ID_MAX_SERVO 6

#define SERVO0  0
#define SERVO1  1
#define SERVO2  2
#define SERVO3  3
#define SERVO4  4
#define SERVO5  5
#define SERVO6  6

#define SERVO0_UP       2600
#define SERVO0_DOWN     1800
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="AX12">
#define AX12_BAUDRATE 500000
#define AX12_BRGVAL 34  //((CLOCK_FREQ_HZ/2/BAUDRATE)/4) - 1 

#define AX12_TX_SIZE 100
#define AX12_RX_SIZE 100

#define AX12_ID_1 0x01
#define AX12_ID_3 0x03
#define AX12_ID_ALL 0xFE

#define AX12_COMMAND_CW_ANGLE_LIMIT_LOW 0x06
#define AX12_COMMAND_CW_ANGLE_LIMIT_HIGH 0x07
#define AX12_COMMAND_CCW_ANGLE_LIMIT_LOW 0x08
#define AX12_COMMAND_CCW_ANGLE_LIMIT_HIGH 0x09
#define AX12_COMMAND_STATUS_RETURN_LEVEL 0x10
#define AX12_COMMAND_LED 0x19
#define AX12_COMMAND_GOAL_POSITION_LOW 0x1E
#define AX12_COMMAND_GOAL_POSITION_HIGH 0x1F
#define AX12_COMMAND_MOVING_SPEED_LOW 0x20
#define AX12_COMMAND_MOVING_SPEED_HIGH 0x21

#define AX12_INSTRUCTION_WRITE_DATA 0x03
// </editor-fold>



#endif	/* CONSTANT_H */
