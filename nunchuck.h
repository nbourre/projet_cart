/* 
 * File:   nunchuck.h
 * Author: nbourre
 *
 * Created on 12 novembre 2016, 09:31
 */

#include "I2C.h"



#ifndef NUNCHUCK_H
#define	NUNCHUCK_H

#ifdef	__cplusplus
extern "C" {
#endif

    
void NC_init(void);
void NC_update(void);
void NC_delay(int);
unsigned char NC_getJoystickX(void);
unsigned char NC_getJoystickY(void);
int NC_getAxisX(void);
int NC_getAxisY(void);
int NC_getAxisZ(void);
unsigned char NC_getButtonZ(void);
unsigned char NC_getButtonC(void);
struct Nunchuck NC_getData(void);


#ifdef	__cplusplus
}
#endif

#endif	/* NUNCHUCK_H */

