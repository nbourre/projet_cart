/*
 * File:   main.c
 * Author: nbourre
 *
 * Created on 5 novembre 2016, 19:07
 */


#include <xc.h>

#define MAX_UNSIGNED_LONG 4294967295
#define PWM_DC_MAX 2000

void initRPs(void);
void initInterrupts(void);
void initTimers(void);
void initTimer1(void);
void initTimer3(void);
void initOC1(void);
void initOC4(void);
void setOC1(int);
void setOC4(int);
void setRightMotorSpeed(int);

volatile long lTicks = 0;
volatile long rTicks = 0;

volatile unsigned long  ms = 0;
volatile unsigned long lWheelAcc = 0;
volatile unsigned long rWheelAcc = 0;

int rwDir = 0;
int pwm_ratio = 16;

int main(void) {
    initRPs();
    initTimers();
    initOC4();
    initInterrupts();
    
    AD1PCFG = 0xFFFF;
    TRISA = 0x0000;
    TRISB = 0xE000; // RP13|14|15 pour PWM1|int1|2
    
    setOC4(0);
    int value = 60;
    setRightMotorSpeed(value);
    while (1) {
        if (rTicks > 25) {
            rTicks = 0;
            if (value > 0) {
                value--;
                setRightMotorSpeed(value);
            }
            
        }
    }
    
    return 0;
}

void initRPs()
{
  // Unlock registers
  asm volatile ( 	"MOV #OSCCON,W1 \n"
  "MOV #0x46,W2	\n"
  "MOV #0x57,W3	\n"
  "MOV.b W2,[W1]	\n"
  "MOV.b W3,[W1]	\n"
  "BCLR OSCCON,#6");
  
  _RP3R = 3;		// Assign TX to RP3 (0x3)
  _U1RXR = 2;
  
  _INT1R = 14;
  _INT2R = 15;
  
  _RP13R = 21; // OC4 Voir p.103, PWM sur pin RP13
  
  
  //RPINR0bits.INT1R4 = 1; // INT0 sur RP15
  //RPINR1 = 0x0E00; // INT1 sur RP14
  
  // Lock registers
  asm volatile ( 	"MOV #OSCCON,W1 \n"
  "MOV #0x46,w2	\n"
  "MOV #0x57,w3	\n"
  "MOV.b W2,[W1]	\n"
  "MOV.b W3,[W1]	\n"
  "BSET OSCCON,#6");
}

void initInterrupts() {
    INTCON2 = 0x0000; // Conf sur front montant
    
    // Page 57
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;
    IPC5bits.INT1IP = 1;
    
    IFS1bits.INT2IF = 0;
    IEC1bits.INT2IE = 1;
    IPC7bits.INT2IP = 1;
}

void initTimers() {
    initTimer1();
    initTimer3();
}

void initTimer1() {
    T1CON = 0x8010;
    PR1 = 999;

    // Démarrage de la minuterie à 1ms
    _T1IF = 0;
    _T1IE = 1;
    _T1IP = 2;
}

// Initialisation du Timer pour PWM pour avoir 4k
void initTimer3() {
    T3CON = 0x8000;
    PR3 = PWM_DC_MAX; // 8M/4K Peripheral register
    
}

void initOC4() {
    OC4CON = 0x000D;
    OC4R = 0;
    OC4RS = 1000;  
}

void setOC4(int value) {
    OC4RS = value;
}


// OCx --> Output Comparator
// Voir page 133 pour les config
void initOC1(void) {
    OC1CON = 0x0005; // Sur TM2 (bit3) voir page 133
    OC1R = 0;
    OC1RS = 1500;
}

void setOC1(int value) {
    OC1RS = value;
}


void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void) {
    // Code ici
    lTicks ++;
    _RB6 = ~_RB6;
    
    
    IFS1bits.INT1IF = 0;
}

void __attribute__((__interrupt__, auto_psv)) _INT2Interrupt(void) {
    // Code ici
    rTicks ++;
    _RB7 = ~_RB7;
    
    IFS1bits.INT2IF = 0;
}

// Toutes les ms
void _ISRFAST __attribute__((auto_psv)) _T1Interrupt(void)
{
    ms = ms < MAX_UNSIGNED_LONG ? ms++: 0;
    
    _T1IF = 0;
}

/** Set the speed of the motor 
 * if value < 127 Rear, 0 <- fast 126 <- slow
 * value > 127 Forward 255 <- fast 128 <- slow
 */
void setRightMotorSpeed(int value) {
    // Dead zone entre 190 - 127 --> 63
    // Force d'inertie
    
    int pwm = PWM_DC_MAX;
    
    if (value < 127) {
        // REVERSE
        _RB12 = 1;
        if (value < 0) value = 0;
        
        pwm = value << 4;
    } else if (value > 127) {
        // FORWARD
        _RB12 = 0;
        
        if (value > 255) value = 255;
        pwm = (value - 127) << 4;
        
    } else {
        // Les deux au max arrête le moteur
        _RB12 = 1;
    }
    
    setOC4 ( pwm );
}

/** Set the speed of the motor 
 * if value < 127 Rear, 0 <- fast 126 <- slow
 * value > 127 Forward 255 <- fast 128 <- slow
 */
void setLefttMotorSpeed(int value) {
    // Dead zone entre 190 - 127 --> 63
    // Force d'inertie
    
    int pwm = PWM_DC_MAX;
    
    if (value < 127) {
        // REVERSE
        _RB12 = 1;
        if (value < 0) value = 0;
        
        pwm = value << 4;
    } else if (value > 127) {
        // FORWARD
        _RB12 = 0;
        
        if (value > 255) value = 255;
        pwm = (value - 127) << 4;
        
    } else {
        // Les deux au max arrête le moteur
        _RB12 = 1;
    }
    
    setOC4 ( pwm );
}