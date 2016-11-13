/* 
 * File:   I2C.h
 * Author: boisclai
 *
 * Created on 7 octobre 2015, 10:57
 */

#ifndef I2C_H
#define	I2C_H

void I2C_Acknowledge(void);
char I2C_Adresse(unsigned char adresse, char lecture);
void I2C_ConditionArret(void);
void I2C_ConditionDemarrage(void);
void I2C_ConditionRedemarrage(void);
unsigned char I2C_EnvoiOctet(unsigned char octet);
void I2C_Initialisation(void);
unsigned char I2C_LireOctet(void);
void I2C_LireOctets(unsigned char tableau[],int nbOctets);
void I2C_NotAcknowledge(void);

#endif	/* I2C_H */

