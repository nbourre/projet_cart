/*
 * File:   main.c
 * Author: nbourre
 *
 * Created on 5 novembre 2016, 19:07
 */


#include <xc.h>


#define MAX_UNSIGNED_LONG 4294967295
#define PWM_DC_MAX 2048
#define NC_DATA_LENGTH 6
#define BUFFER_SIZE 12
#define SPEED_MAX 256
#define CRLF sendChar('\r');sendChar('\n');


_CONFIG1(ICS_PGx2 & JTAGEN_OFF & GCP_OFF & GWRP_OFF & FWDTEN_OFF)
_CONFIG2(0x7987)

typedef enum {
    RUNNING_MODE, 
    CRUISECONTROL_MODE,
    CONFIG_MODE,
            IDLE_MODE,
            AUTO_MODE,
} cartState;

cartState currentCartState = RUNNING_MODE;        


        
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
void motorSetRawSpeeds(int);

void setCartState(cartState);

void sendChar(unsigned char);
void sendChars();
void sendValue (long value);
void sendCurrentState(void);
void putsUART1(char*);


// prototypes du système
void modeCruising(void);
void modeRunning(void);
void modeConfig(void);
void modeIdle(void);

void manageSystem(void);
void manageComm(void);
void manageInterrupts(void);


// Cruise control variables
int cruiseControlSpeed = 0;
int isCruiseControlled = 0;
int currentGasValue = 0;


// Left wheel variables
volatile long lTicks = 0;
volatile unsigned long lwAcc = 0;
volatile unsigned long lwDeltaTime = 0;
long lwCruiseDT = 0;
int lwCruiseTarget = 128;
int lwEncoderInt = 0;
int lwCurrentSpeed = 128;
long lwAverage = 0;

// Right wheel variables
volatile long rTicks = 0;
volatile unsigned long rwAcc = 0;
volatile unsigned long rwDeltaTime = 0;
long rwCruiseDT = 0;
int rwCruiseTarget = 128;
int rwEncoderInt = 0;
int rwCurrentSpeed = 128;
long rwAverage = 0; // Moyenne en ms du délai entre 2 ticks à 80% en E10x-6 s

int rwSlower = 1; // Indique au système que la roue gauche doit s'ajuster

// Auto-mode variables
int ticksToGoal = 240;
int amStartSpeed = 60;
int amRampAcc = 0;
int amRampDelay = 10; // 

// Calibaration var
int speedCalibFlag = 0;
int speedCalibAcc = 0;
int speedCalibDelay = 3000;

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

unsigned char rxStart = 0;
unsigned char rxCount = 0;
unsigned char rxBuffer[2];
unsigned char isDataReady = 0;
unsigned long rxAcc = 0;
int rxChickenSwitch = 50;
unsigned char rxOk = 1;

int blinkIdleDelay = 100;

// System variables
volatile unsigned long nb_ms = 0;
unsigned long runningTime = 0;

int main(void) {
    initRPs();
    initTRISx();
    initTimers();

    initOCx();
    initInterrupts();
    initRxTx();
    
    lowRangeRatio = (y_mid - y_min) / 128.0;
    hiRangeRatio = 128.0 / (y_max - y_mid);

    

    allMotorStop();
    
    int calibRun = 5;

    while (1) {
        
        if (calibRun > 0) {
            calibRun--;
            allMotorStop();
        } else {

            manageInterrupts();

            manageSystem();
            manageComm();
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
    lwEncoderInt = 1;    
    
    IFS1bits.INT1IF = 0;
}

/** Interruption sur encodeur droit  */
void __attribute__((__interrupt__, auto_psv)) _INT2Interrupt(void) {
    rwEncoderInt = 1;
    
    IFS1bits.INT2IF = 0;
}

// Toutes les ms
void _ISRFAST __attribute__((auto_psv)) _T1Interrupt(void)
{
    if(nb_ms > 0)   --nb_ms;
    runningTime++;
    blinkAcc++;
    commTicks++;
    rwAcc++;
    lwAcc++;
    rxAcc++;
    
    if (currentCartState == CONFIG_MODE) {
        if (speedCalibFlag) {
            speedCalibAcc++;
        }
    } else if (currentCartState == AUTO_MODE) {
        amRampAcc++;
    }
    
    
    
    _T1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{
    rxChar = U1RXREG;   //Appel la fonction de lecture
    
    if (rxAcc < rxChickenSwitch) {
        rxAcc = 0;
        rxOk = 1;
    }
    
    
    if (currentCartState == CONFIG_MODE ) {
//        if (rxChar == 's') {
//            rwDeltaTime = lwDeltaTime = 0;
//            speedCalibFlag = 1;
//            rTicks = 0;
//            lTicks = 0;
//            rwAcc = 0;
//            lwAcc = 0;
//        }
    } else if (currentCartState == RUNNING_MODE) {
        if (rxStart == 0) {
            if (rxChar == 0x55) {
                // 0x55 == 'U'
                rxStart = 1;
            }
        } else {
            rxBuffer[rxCount++] = rxChar;
            
            if (rxCount > 1) {
                rxCount = 0;
                rxStart = 0;
                isDataReady = 1;
            }
        }
    } else if (currentCartState == IDLE_MODE) {
        rxAcc = 0;
        rxStart = 0;
        rxCount = 0;
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

void motorSetRawSpeeds(int value) {
    rightMotorSetSpeed(value);
    leftMotorSetSpeed(value);
}

void motorSetSpeed(int value) {
    
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
        
        setCartState(RUNNING_MODE);
    }
    
    
    if (blinkAcc > 100) {
        blinkAcc = 0;
        _RB7 = ~_RB7;
        _RB6 = ~_RB6;
    }
}

void modeIdle() {
    if (rxOk) {
        setCartState(RUNNING_MODE);
    }
    
    allMotorStop();
    
    if (blinkAcc > blinkIdleDelay) {
        blinkAcc = 0;
        _RB7 = ~_RB7;
        _RB6 = ~_RB6;
    }
}

void modeAuto() {
    
    if (rTicks < ticksToGoal) {
        if (amRampAcc > amRampDelay) {
            amRampAcc = 0;
            
            if (rTicks  < ticksToGoal >> 1) {
                if (amStartSpeed > 0) amStartSpeed--;
            } else {
                if (amStartSpeed < 80) amStartSpeed++;
            }
        }
        
        rightMotorSetSpeed(amStartSpeed);
        leftMotorSetSpeed(amStartSpeed);
    } else {
        rTicks = 0;
        allMotorStop();
        setCartState(RUNNING_MODE);
    }
    
    if (blinkAcc > 250) {
        blinkAcc = 0;
        _RB7 = ~_RB7;
        _RB6 = ~_RB6;
    }
}

void modeRunning() {
    
    if (rxAcc > rxChickenSwitch) {
        rxAcc = 0;
        rxOk = 0;
        
        allMotorStop();
        
        _RB6 = 1;
        _RB7 = 0;
        
        setCartState(IDLE_MODE);       
    }   

    // Gestion des données reçues par RX
    if (isDataReady) {
        isDataReady = 0;
        
        if ((rxBuffer[0] == '!' && rxBuffer[1] == 'x') ||
            (rxBuffer[0] == 0 && rxBuffer[1] == 0)) {
            allMotorStop();
            rTicks = 0; 
            lTicks = 0;
            _RB6 = 1;
            _RB7 = 0;
            amRampAcc = 0;
            setCartState(AUTO_MODE);
        } 

        else {
            rightMotorSetSpeed(rxBuffer[0]);
            leftMotorSetSpeed(rxBuffer[1]);
        }
        
        
        rxBuffer[0] = rxBuffer[1] = 128;
    }
}



void setCartState(cartState newState) {
    currentCartState = newState;
    sendCurrentState();
}

void modeConfig() {
    
    // Faire clignoter les leds pour indiquer le mode calibration
    if (blinkAcc > 500) {
        blinkAcc = 0;
        _RB7 = ~_RB7;
        _RB6 = ~_RB6;
    }
    
    
    if (speedCalibFlag) {
        motorSetRawSpeeds(SPEED_MAX * 0.90);
        
        if (speedCalibAcc > speedCalibDelay) {
            speedCalibFlag = 0;
            speedCalibAcc = 0;
            allMotorStop();
             
            // Calculer le DT moyen des moteurs
            // Diviser multiplier par 1000 étant donné que l'on travaille
            // avec des longs
            rwAverage = (rwDeltaTime * 1.0) / rTicks * 1000;
            lwAverage = (lwDeltaTime * 1.0) / lTicks * 1000;
            
            isOkToSend = 1;
            
            
        }
        
        if (speedCalibFlag == 0) {
            rwSlower = rwAverage > lwAverage ? 1 : 0;
        }
    }
    
}

void manageSystem() {
    
    rwCurrentSpeed = rightMotorGetSpeed();
    lwCurrentSpeed = leftMotorGetSpeed();
    

    
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
        case IDLE_MODE:
            modeIdle();
            break;
        case AUTO_MODE:
            modeAuto();
        default:
            break;
    }
}


void manageComm() {
    if (currentCartState == CONFIG_MODE) {
        if (isOkToSend) {
            isOkToSend = 0;
            
            sendChar('r');
            sendChar('w');
            sendChar('=');
            sendValue(rwAverage);
            sendChar('l');
            sendChar('w');
            sendChar('=');
            sendValue(lwAverage);
        }
    } else {

        if (commTicks > 999) {
            commTicks = 0;
            sendValue(runningTime);
        }
    }
}

void manageInterrupts() {
    if (rwEncoderInt) {
        rwEncoderInt = 0;
        
         if (currentCartState == RUNNING_MODE) {
            rTicks += rwCurrentSpeed > 128 ? 1 : -1;

            rwDeltaTime = rwAcc;
            rwAcc = 0;

            if (isCruiseControlled) {
//                if (rwCruiseTarget > nunchuck.jy) {
//                    if (rwCruiseDT != 0) {
//                        if (rwDeltaTime > rwCruiseDT) {
//                            ++OC4RS;
//                        } else {
//                            --OC4RS;
//                        }        
//                    }
//                }
            }
        } else if (currentCartState == CONFIG_MODE) {
            rTicks++;
            rwDeltaTime += rwAcc;
            rwAcc = 0;
            
        } else if (currentCartState == AUTO_MODE) {
            rTicks++;
            rwDeltaTime += rwAcc;
            rwAcc = 0;
        }
    }
    
    if (lwEncoderInt) {
        lwEncoderInt = 0;
        
        
        if (currentCartState == RUNNING_MODE) {
            lTicks += lwCurrentSpeed > 128 ? 1 : -1;
            //_RB6 = ~_RB6;    

            lwDeltaTime = lwAcc;
            lwAcc = 0;

            if (isCruiseControlled) {
//                if (lwCruiseTarget > nunchuck.jy) {
//                    if (lwCruiseDT != 0) {
//                        if (lwDeltaTime > lwCruiseDT) {
//                            ++OC1RS;
//                        } else {
//                            --OC1RS;
//                        }        
//                    }
//                }
            }
        } else if (currentCartState == CONFIG_MODE) {
            lTicks++;
            lwDeltaTime += lwAcc;
            lwAcc = 0;
        } else if (currentCartState == AUTO_MODE) {
            lTicks++;
            lwDeltaTime += lwAcc;
            lwAcc = 0;
        }
        
    }
    
    
}

void Delai(int ms)
{
  nb_ms = ms;
  while(nb_ms > 0);
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
    // Configuration du port série (UART)
    U1BRG  = 51; // 
    //U1BRG = 0;

    U1STA  = 0x2000;	// Interruption à chaque caractère reçu
    U1MODE = 0x8000;    // BRGH = 0

    U1STAbits.UTXEN = 1;

    _U1RXIP = 2; // Priorite
    _U1RXIF = 0;
    _U1RXIE = 1; 

}

void initTRISx() {
    AD1PCFG = 0xFFFF;
    TRISA = 0x0000;
    
    TRISB = 0xC004; // 14|15 pour int1|2 soit encRight, encLeft

}

void sendCurrentState() {
    switch (currentCartState) {
        case RUNNING_MODE:
            sendChar('M');
            sendChar('R');
            sendChar('U');
            sendChar('N');
            CRLF
            break;
        case IDLE_MODE:
            sendChar('M');
            sendChar('I');
            sendChar('D');
            sendChar('L');
            CRLF
            break;
        case CONFIG_MODE:
            sendChar('M');
            sendChar('C');
            sendChar('F');
            sendChar('G');            
            CRLF
            break;
        case CRUISECONTROL_MODE:
            sendChar('M');
            sendChar('C');
            sendChar('C');
            sendChar('T');
            sendChar('R');
            sendChar('L');
            CRLF
            break;
        case AUTO_MODE:
            sendChar('A');
            sendChar('M');
            CRLF
            break;
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
     
     char temp;
     do                             // write least significant digit of value that's left
     {
         temp = (value % 10) + '0';
         buffer[--c] = temp;
         value /= 10;
     } while (value);
 
     if (original < 0) 
         buffer[--c] = '-';
 
     return &buffer[c];
 }
 
 // TODO : Delete
 void sendChars() {
    int i;
    for (i = 0; i < BUFFER_SIZE; i++) {
        if (textBuffer[i] != 0){
//        if (textBuffer[i] >= '0' && textBuffer[i] <= '9' || textBuffer[i] == '-') {
            sendChar(textBuffer[i]);
        }
    }
    sendChar('\r');
    sendChar('\n');
    
    
}
 
 void sendValue (long value) {
     textBuffer = itoa(value);
     putsUART1(textBuffer);
     CRLF;
 }
 
void putsUART1(char* buffer){   
    char * temp_ptr =  buffer;   
    while(*temp_ptr != '\0')        {       
        while(U1STAbits.UTXBF);  /* wait if the buffer is full */
         U1TXREG = *temp_ptr++;   /* transfer data byte to TX reg */          
    }
}