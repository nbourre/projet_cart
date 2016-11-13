#include <xc.h>

void I2C_Acknowledge()
{
    I2C1CONbits.ACKDT = 0;
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN);
}

char I2C_Adresse(unsigned char adresse, char lecture)
{
    I2C1TRN = (adresse << 1) | lecture;
    while (I2C1STATbits.TRSTAT || I2C1STATbits.TBF);

    return I2C1STATbits.ACKSTAT;
}

void I2C_ConditionArret()
{
    I2C1CONbits.PEN = 1;
    while(I2C1CONbits.PEN);
}

void I2C_ConditionDemarrage()
{
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN);
}

void I2C_ConditionRedemarrage()
{
    I2C1CONbits.RSEN = 1;
    while(I2C1CONbits.RSEN);
}

unsigned char I2C_EnvoiOctet(unsigned char octet)
{
    I2C1TRN = octet;
    while (I2C1STATbits.TRSTAT || I2C1STATbits.TBF);

    return I2C1STATbits.ACKSTAT;
}

void I2C_Initialisation()
{
    I2C1BRG = 0x4F;
    I2C1CONbits.I2CEN = 1;
}

unsigned char hex[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

unsigned char I2C_LireOctet()
{
    I2C1CONbits.RCEN = 1;
    while (I2C1CONbits.RCEN);
    return I2C1RCV;
}

void I2C_LireOctets(unsigned char tableau[],int nbOctets)
{
    int i;

    if(nbOctets < 1)    return;

    for(i=0;i<nbOctets-1;++i)
    {
        tableau[i] = I2C_LireOctet();
        I2C_Acknowledge();
    }
    tableau[i] = I2C_LireOctet();
    I2C_NotAcknowledge();
}

void I2C_NotAcknowledge()
{
    I2C1CONbits.ACKDT = 1;
    I2C1CONbits.ACKEN = 1;
    while (I2C1CONbits.ACKEN);
}





