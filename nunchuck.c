/*
 * File:   nunchuck.c
 * Author: nbourre
 *
 * Created on 12 novembre 2016, 10:03
 */

#include <xc.h>
#include "I2C.h"

#define NC_DATA_LENGTH 6

struct Nunchuck {
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

unsigned char nc_data[NC_DATA_LENGTH];
int nc_nb_ms = 0;


void NC_init(){
    I2C_Initialisation();
    
    // Mettre délai de 50 ms;
    
    I2C_ConditionDemarrage();
    I2C_Adresse(0x52, 0);
    I2C_EnvoiOctet(0xF0);       // Sequence compatible aux deux modeles sans encodage
    I2C_EnvoiOctet(0x55);
    I2C_ConditionArret();
    
    // Mettre délai de 10 ms;
    
    I2C_ConditionDemarrage();
    I2C_Adresse(0x52, 1);
    I2C_LireOctets(nc_data, NC_DATA_LENGTH);
    I2C_ConditionArret();
}

void NC_delay(int ms){
    
}

struct Nunchuck NC_getData() {
    struct Nunchuck result;   
    
//    jx = donnees[0];
//    jy = donnees[1];
//    ax = (donnees[2] << 2) + ((donnees[5] & 0x0C) >> 2);
//    ay = (donnees[3] << 2) + ((donnees[5] & 0x30) >> 4);    
//    az = (donnees[4] << 2) + ((donnees[5] & 0xC0) >> 6);
//    
//    bz = !(donnees[5] & 0x01);
//    bc = !((donnees[5] & 0x02) >> 1);
        
    result.jx = nc_data[0];
    result.jy = nc_data[1];
    result.ax = (nc_data[2] << 2) + ((nc_data[5] & 0x0C) >> 2);
    result.ay = (nc_data[3] << 2) + ((nc_data[5] & 0x30) >> 4);    
    result.az = (nc_data[4] << 2) + ((nc_data[5] & 0xC0) >> 6);
    
    result.bz = !(nc_data[5] & 0x01);
    result.bc = !((nc_data[5] & 0x02) >> 1);
    
    return result;
}
