/*
 * File:   main.c
 * Author: nbourre
 *
 * Created on 5 novembre 2016, 19:07
 */


#include <xc.h>

#include "I2C.h"
//#include "I2C.h"
//#include "nunchuck.h"

#define MAX_UNSIGNED_LONG 4294967295
#define PWM_DC_MAX 2048
#define NC_DATA_LENGTH 6
#define BUFFER_SIZE 12
#define LOCAL_NUNCHUCK 0

_CONFIG1(ICS_PGx2 & JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_OFF)
_CONFIG2(0x7987)

typedef enum {
    RUNNING_MODE, 
    CRUISECONTROL_MODE,
    CONFIG_MODE            
} cartState;

cartState currentCartState = RUNNING_MODE;        

struct Nunchuck {
    int active;
    // Axes x, y, z
    int az; 
    int ay;
    int ax;
    
    // Joystick x, y
    unsigned char jy;
    unsigned char jx;
    
    // Button Z, C
    int bz;
    int bc;
};

volatile struct Nunchuck nunchuck;
        
// prototypes helpers
void initRPs(void);
void initInterrupts(void);
void initRxTx(void);
void initTimers(void);
void initTimer1(void);
void initTimer3(void);
void initOCx(void);
void initTRISx(void);
char *itoa(long);

void setOC1(int);
void setOC4(int);
void rightMotorSetSpeed(int);
void rightMotorStop(void);
int rightMotorGetSpeed(void);

void leftMotorStop();
void leftMotorSetSpeed(int);
int leftMotorGetSpeed(void);

void allMotorStop();

void sendChar(unsigned char);
void sendChars();
void testLed(void);

// prototypes de la NUNCHUCK
void nunchuckInit(void);
void nunchuckUpdate(void);
struct Nunchuck nunchuckConvertRawData(unsigned char*);
void nunchuckSendToPC(void);

// prototypes du système
void modeCruising(void);
void modeRunning(void);
void modeConfig(void);
void calibrate(void);

void manageInputs(void);
void manageSystem(void);
void manageComm(void);
void manageInterrupts(void);


int cruiseControlSpeed = 0;
int isCruiseControlled = 0;
int currentGasValue = 0;
int setNewSpeed = 0;

// Left wheel variables
volatile long lTicks = 0;
volatile unsigned long lWheelAcc = 0;
volatile unsigned long lwDeltaTime = 0;
long lwCruiseDT = 0;
int lwValue = 128;

// Right wheel variables
volatile long rTicks = 0;
volatile unsigned long rWheelAcc = 0;
volatile unsigned long rwDeltaTime = 0;
long rwCruiseDT = 0;
int rwValue = 128;

volatile unsigned long nb_ms = 0;



int btnCnZPressTime = 0;
int btnZPressTime = 0;
int btnCPressTime = 0;

int blinkAcc = 0;
int x_min = 22;
int x_max = 218;
int x_mid = 124;

int y_min = 34;
int y_max = 223;
int y_mid = 135;

double slopeX;
double slopeY;
double lowRangeRatio;
double hiRangeRatio;

int rwDir = 0;
int pwm_ratio = 16;

// Données de la NC
unsigned char nc_data[NC_DATA_LENGTH];

// Communication
unsigned char rxChar;
int forward = 0;
char *textBuffer;
int isOkToSend = 0;
int commTicks = 0;


int main(void) {
    initRPs();
    initTRISx();
    initTimers();

    initOCx();
    initInterrupts();
    initRxTx();
    
    lowRangeRatio = (y_mid - y_min) / 128.0;
    hiRangeRatio = 128.0 / (y_max - y_mid);

    
    nunchuckInit();

    allMotorStop();

    while (1) {

        manageInterrupts();
        
        manageInputs();
        manageSystem();
        manageComm();


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
  
  _INT1R = 14;      // INT1 sur RP14 pour encodeur droit
  _INT2R = 15;      // INT2 sur RP15 pour encodeur gauche
  
  _RP13R = 21; // OC4 Voir p.103, PWM sur pin RP13 pour moteur droit
  _RP11R = 18; // OC1 Voir p.103, PWM sur pin RP12 pour moteur droit
  
  
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
    INTCON2 = 0x0000; // Tous config. sur front montant
    
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

// Toutes les ms
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
    T3CON = 0x8000; // Full vitesse sans prescaler
    PR3 = PWM_DC_MAX; // 8M/4K Peripheral register
}


void setOC4(int val) {
    OC4RS = val;
}




// OCx --> Output Comparator
// Voir page 133 pour les config
void initOCx(void) {
    OC1CON = 0x000D; // PWM Sur TM3 (bit3) voir page 133
    OC1R = 0;
    OC1RS = 1;
    
    OC4CON = 0x000D; // PWM continue sur timer3
    OC4R = 0;
    OC4RS = 1;
    
}

void setOC1(int value) {
    OC1RS = value;
}

/** Interruption sur encodeur gauche */
void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void) {
    // Code ici
    if (currentCartState == CONFIG_MODE) {
    
    } else {
        lTicks ++;
        //_RB6 = ~_RB6;    
        
        lwDeltaTime = lWheelAcc;
        lWheelAcc = 0;

        if (isCruiseControlled) {
            if (lwValue > nunchuck.jy) {
                if (lwCruiseDT != 0) {
                    if (lwDeltaTime > lwCruiseDT) {
                        ++OC1RS;
                    } else {
                        --OC1RS;
                    }        
                }
            }
        }
    }
    
    
    
    IFS1bits.INT1IF = 0;
}

/** Interruption sur encodeur droit  */
void __attribute__((__interrupt__, auto_psv)) _INT2Interrupt(void) {
    // Code ici
    
    if (currentCartState == CONFIG_MODE) {
        
    } else {
        //_RB7 = ~_RB7; // Faire clignoter le LED à chaque passage
        rTicks ++;

        rwDeltaTime = rWheelAcc;
        rWheelAcc = 0;

        if (isCruiseControlled) {
            if (rwValue > nunchuck.jy) {
                if (rwCruiseDT != 0) {
                    if (rwDeltaTime > rwCruiseDT) {
                        ++OC4RS;
                    } else {
                        --OC4RS;
                    }        
                }
            }
        }

        //isOkToSend = 1;
        //rwCruiseSpeed = rwDeltaTime;
    }
    
    IFS1bits.INT2IF = 0;
}

// Toutes les ms
void _ISRFAST __attribute__((auto_psv)) _T1Interrupt(void)
{
    if(nb_ms > 0)   --nb_ms;
    blinkAcc++;
    commTicks++;
    
    if (currentCartState == CONFIG_MODE) {
        
    } else {
        rWheelAcc++;
        lWheelAcc++;
    }
    
    if (nunchuck.bc && nunchuck.bz) {
        btnCnZPressTime++;
    } else {
        btnCnZPressTime = 0;
    }
    
    if (nunchuck.bz) {
        btnZPressTime++;
    } else {
        btnZPressTime = 0;
    }
    
    if (nunchuck.bc) {
        btnCPressTime++;
    } else {
        btnCPressTime = 0;
    }
    
    
    
    _T1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    rxChar = U1RXREG;   //Appel la fonction de lecture
    
    _RB7 = ~_RB7;
    
    if (rxChar == 'w') {
        forward = ~forward;
    }
    
    _U1RXIF = 0;
}

/** Set the speed of the motor 
 * if value < 128 Rear, 0 <- fast 127 <- slow
 * value > 128 Forward 256 <- fast 129 <- slow
 */
void rightMotorSetSpeed(int value) {
    // Dead zone entre 190 - 128 --> 63
    // Force d'inertie
    
    int pwm = PWM_DC_MAX;
    
    /** Correction de la valeur*/
//    int diff = value - y_mid;
//    int corrValue = value;
//    
//    if (diff > -10 || diff < 10) {
//        corrValue = 128;
//    } else {
//        if (diff < 0) {
//            corrValue = lowRangeRatio * value;
//        } else {
//            corrValue = hiRangeRatio * value;
//        }     
//    }
    
    if (value < 128) {
        // REVERSE
        _RB12 = 1;
        if (value < 1) value = 1;
        
        pwm = PWM_DC_MAX - ((128 - value) << 4);
    } else if (value > 128) {
        // FORWARD
        _RB12 = 0;
        
        if (value > 256) value = 256;
        pwm = (value - 128) << 4;
        
    } else {
        // Les deux au max arrête le moteur        
        _RB12 = 1; // LN2
    }
    
    setOC4 ( pwm );
}

int rightMotorGetSpeed() {
    if (!_RB12) {    
        return (OC4RS >> 4) + 128;
    } else {
        return 128 - ((PWM_DC_MAX - OC4RS) >> 4);
    }    
}

int leftMotorGetSpeed() {
    if (!_RB10) {    
        return (OC4RS >> 4) + 128;
    } else {
        return 128 - ((PWM_DC_MAX - OC4RS) >> 4);
    }    
}


void rightMotorStop() {
    rightMotorSetSpeed(128);
}

void leftMotorStop() {
    leftMotorSetSpeed(128);
}

/** Set the speed of the motor 
 * if value < 127 Rear, 0 <- fast 126 <- slow
 * value > 127 Forward 255 <- fast 128 <- slow
 */
void leftMotorSetSpeed(int value) {
    // Dead zone entre 190 - 127 --> 63
    // Force d'inertie
    
    int pwm = PWM_DC_MAX;
    
    
    if (value < 128) {
        // REVERSE
        _RB10 = 1;
        if (value < 1) value = 1;
        
        pwm = PWM_DC_MAX - ((128 - value) << 4);
    } else if (value > 128) {
        // FORWARD
        _RB10 = 0;
        
        if (value > 256) value = 256;
        pwm = (value - 128) << 4;
        
    } else {
        // Les deux au max arrête le moteur        
        _RB10 = 1;
    }
    
    setOC1 ( pwm );
}

void allMotorStop(){
    rightMotorStop();
    leftMotorStop();
}


/**
 * Permet de gérer le 
 */
void modeCruising() {
    if (btnCPressTime > 10) {
        btnCPressTime = 0;
        
        isCruiseControlled = 0;
        rightMotorStop();
        
        currentCartState = RUNNING_MODE;
    }
    
    if (nunchuck.active) {
        if (rwValue < nunchuck.jy) {
            rightMotorSetSpeed(nunchuck.jy * hiRangeRatio);
        }

        if (lwValue < nunchuck.jy) {
            leftMotorSetSpeed(nunchuck.jy * hiRangeRatio);
        }
    }
    
    
    if (blinkAcc > 100) {
        blinkAcc = 0;
        _RB7 = ~_RB7;
        _RB6 = ~_RB6;
    }
}

void modeRunning() {
    // Transition vers etat de configuration
    if (btnCnZPressTime > 1000) {
        btnCnZPressTime = 0;
        
        currentCartState = CONFIG_MODE;
        _RB7 = 1;
        _RB6 = 1;
        allMotorStop();
    }
    
    if (btnZPressTime > 250) {
        isCruiseControlled = 1;
        
        rwCruiseDT = rwDeltaTime;
        lwCruiseDT = lwDeltaTime;
        
        rwValue = rightMotorGetSpeed();
        lwValue = leftMotorGetSpeed();
        
        
        btnZPressTime = 0;
        currentCartState = CRUISECONTROL_MODE;
    }
    
    if (nunchuck.active) {
        if (nunchuck.jy - y_mid < 5 && nunchuck.jy - y_mid > -5 ) {
            rightMotorStop();
            leftMotorStop();
        } else {
            if (nunchuck.jy > y_mid) {
                rightMotorSetSpeed(nunchuck.jy * hiRangeRatio);
                leftMotorSetSpeed(nunchuck.jy * hiRangeRatio);
            } else {
                rightMotorSetSpeed(nunchuck.jy * lowRangeRatio);
                leftMotorSetSpeed(nunchuck.jy * lowRangeRatio);
            }
        }    
    }
    
    if (forward) {
        rightMotorSetSpeed(200);
        leftMotorSetSpeed(200);
    } else {
        rightMotorStop();
        leftMotorStop();
    }
    
}

void calibrate() {
    slopeX = 1.0 * 256 / ((x_max - x_min) + 1);
    
    slopeY = 1.0 * 256 / ((y_max - y_min) + 1);
}

void modeConfig() {
    // Transition vers etat d'execution
    if (btnCnZPressTime > 1000) {
        btnCnZPressTime = 0;
        
        currentCartState = RUNNING_MODE;
    }
    
    
    
    if (blinkAcc > 500) {
        blinkAcc = 0;
        _RB7 = ~_RB7;
        _RB6 = ~_RB6;
    }
    
    int newVal = 0;
    
    if (nunchuck.active) {
        if (nunchuck.jx < x_min) {
            x_min = nunchuck.jx;
            newVal = 1;
        }

        if (nunchuck.jx > x_max) {
            x_max = nunchuck.jx;
            newVal = 1;
        }

        if (nunchuck.jy < y_min) {
            y_min = nunchuck.jy;
            newVal = 1;
        }

        if (nunchuck.jy > y_max) {
            y_max = nunchuck.jy;
            newVal = 1;
        }
    }
    
//    if (newVal) {
//        newVal = 0;
//        calibrate();
//        isOkToSend = 1;
//    }
//    
//    if (isOkToSend) {
//        isOkToSend = 0;
//        
//        sendChar('m'); sendChar('i'); sendChar('n'); sendChar('=');
//        textBuffer = itoa(y_min);
//        sendChars();
//        
//        sendChar('m'); sendChar('a'); sendChar('x'); sendChar('=');
//        textBuffer = itoa(y_max);
//        sendChars();
//    }
}

void manageSystem() {
    switch (currentCartState) {
        case CRUISECONTROL_MODE:
            modeCruising();
            break;
        case RUNNING_MODE:
            modeRunning();
            break;
        case CONFIG_MODE:
            modeConfig();
            break;
        default:
            break;
    }
}

void manageInputs() {
#if LOCAL_NUNCHUCK
    nunchuckUpdate();

    nunchuckSendToPC();

    nunchuck = nunchuckConvertRawData(nc_data);
#endif            
}

void manageComm() {
    if (commTicks > 999) {
        commTicks = 0;
        sendChar('Y');
        sendChar('\r');
        sendChar('\n');
        _RB6 = ~_RB6; // Fonctionne pas 2016-11-20
    }
}

void manageInterrupts() {

}

void Delai(int ms)
{
  nb_ms = ms;
  while(nb_ms > 0);
}

void nunchuckInit() {
    nunchuck.active = LOCAL_NUNCHUCK;
#if LOCAL_NUNCHUCK
    I2C_Initialisation();
        
    Delai(50);
    
    I2C_ConditionDemarrage();
    I2C_Adresse(0x52, 0);
    I2C_EnvoiOctet(0xF0);       // Sequence compatible aux deux modeles sans encodage
    I2C_EnvoiOctet(0x55);
    I2C_ConditionArret();
    
    Delai(10);
    
    I2C_ConditionDemarrage();
    I2C_Adresse(0x52, 1);
    I2C_LireOctets(nc_data, NC_DATA_LENGTH);
    I2C_ConditionArret();
    
#endif
}

void nunchuckUpdate() {
    Delai(1);
    
    I2C_ConditionDemarrage();
    I2C_Adresse(0x52, 0);
    I2C_EnvoiOctet(0x00);
    I2C_ConditionArret();
    
    Delai(1);
    
    I2C_ConditionDemarrage();
    I2C_Adresse(0x52, 1);
    I2C_LireOctets(nc_data,6);
    I2C_ConditionArret();
    
}

struct Nunchuck nunchuckConvertRawData(unsigned char data[6]) {
    struct Nunchuck result;   
    
//    jx = donnees[0];
//    jy = donnees[1];
//    ax = (donnees[2] << 2) + ((donnees[5] & 0x0C) >> 2);
//    ay = (donnees[3] << 2) + ((donnees[5] & 0x30) >> 4);    
//    az = (donnees[4] << 2) + ((donnees[5] & 0xC0) >> 6);
//    
//    bz = !(donnees[5] & 0x01);
//    bc = !((donnees[5] & 0x02) >> 1);
        
    result.jx = data[0];
    result.jy = data[1];
    result.ax = (data[2] << 2) + ((data[5] & 0x0C) >> 2);
    result.ay = (data[3] << 2) + ((data[5] & 0x30) >> 4);    
    result.az = (data[4] << 2) + ((data[5] & 0xC0) >> 6);
    
    result.bz = !(data[5] & 0x01);
    result.bc = !((data[5] & 0x02) >> 1);
    
    return result;
}


void sendChar(unsigned char c)
{
  while(U1STAbits.UTXBF);
  U1TXREG = c;
}

void nunchuckSendToPC() {

    // Communication avec GRAccel.exe
    sendChar(0x5A);
    int i;
    
    for(i = 0; i < NC_DATA_LENGTH; ++i) {
      sendChar(nc_data[i]);
    }

}

void initRxTx() {
#if LOCAL_NUNCHUCK
    // Configuration du port série (UART)
    U1BRG  = 16;	// BRGH=1 16=115200
    U1STA  = 0x2000;	// Interruption à chaque caractère reçu
    U1MODE = 0x8008;	// BRGH = 1

    U1STAbits.UTXEN = 1;
#else
    // Configuration du port série (UART)
    U1BRG  = 51; // 
    //U1BRG = 0;

    U1STA  = 0x2000;	// Interruption à chaque caractère reçu
    U1MODE = 0x8000;    // BRGH = 0

    U1STAbits.UTXEN = 1;

    _U1RXIP = 2; // Priorite
    _U1RXIF = 0;
    _U1RXIE = 1; 
#endif
}

void initTRISx() {
    AD1PCFG = 0xFFFF;
    TRISA = 0x0000;
    
    //TRISB = 0xC004; // 14|15 pour int1|2 soit encRight, encLeft
    TRISB = 0x0004;
}

void testLed() {
    if (rTicks > 100) {
        _RB7 = ~_RB7;
    }
}

// Source : http://www.microchip.com/forums/m481146.aspx
// this routine found online somewhere, then tweaked
 // returns pointer to ASCII string in a static buffer
 char *itoa(long value) 
 {
     char buffer[BUFFER_SIZE];        // 12 bytes is big enough for an INT32
     
     int i = 0;
     for (i = 0; i < BUFFER_SIZE; i++) {
         buffer[i] = '*';
     }
     
     long original = value;        // save original value
 
     int c = sizeof(buffer) - 1;
     
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
 
 void sendChars() {
    int i;
    for (i = 0; i < BUFFER_SIZE; i++) {
        if (textBuffer[i] >= '0' && textBuffer[i] <= '9' || textBuffer[i] == '-') {
            sendChar(textBuffer[i]);
        }
    }
    sendChar('\n');
    sendChar('\r');
    
}