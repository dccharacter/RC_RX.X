#include <xc.h>
#include "i2c.h"
#include "main.h"
#include "usart.h"

/* Local defenitions */
void I2C_WaitIdle(void);
void I2C_Ack();
void I2C_Nack();

uint16_t i2c_to;

void I2C_Ack(void) {
    SSP1CON2bits.ACKDT = 0;
    //I2C_WaitIdle();
    SSP1CON2bits.ACKEN = 1; 
}

void I2C_Nack(void) {
    SSP1CON2bits.ACKDT = 1;
    //I2C_WaitIdle();
    SSP1CON2bits.ACKEN = 1; 
}

void I2C_WaitIdle(void) {
    /*
    if (SSP1CON1bits.SSPOV) {
        serialPutS("OVERFLOW\r\n");
        while(1);
    }
    while ((SSP1CON2 & 0x1F) || (SSP1STATbits.R_nW));*/
    /* wait for any pending transfer */
    while(!SSP1IF);
    SSP1IF = 0;
}

uint8_t I2C_GetByte() {
    volatile unsigned char temp;
    /* Reception works if transfer is initiated in read mode */
    SSP1CON2bits.RCEN = 1; /* Enable data reception */
    I2C_WaitIdle();
    //while (!SSP1STATbits.BF);
    temp = SSP1BUF; /* Read serial buffer and store in temp register */
    I2C_Nack();
    return temp; /* Return the read data from bus */
}

void I2C_MultiRead(uint8_t *pBuffer, uint16_t NumByteToRead) {
    while (NumByteToRead--) {
        SSP1CON2bits.RCEN = 1;
        I2C_WaitIdle();
        *pBuffer++ = SSP1BUF;
        if (NumByteToRead) {
            I2C_Ack();
        } else {
            I2C_Nack();
        }
    }
}

void I2C_MultiWrite(uint8_t * pBuffer) {
    while (*pBuffer) {
        I2C_SendByte(*pBuffer++);
    }
}

ACK_STATUS I2C_SendByte(uint8_t Byte) {
    SSP1BUF = Byte;
    I2C_WaitIdle();
    ACK_STATUS result = SSP1CON2bits.ACKSTAT;
    if (result) {
        serialPutS("I2C error\r\n");
    } else {
        serialPutS("Sent OK\r\n");
    }
    return result;
}

void I2C_GenerateStart(void) {   
        LED=1;
    __delay_us(5);
    LED=0;
    SSP1CON2bits.SEN = 1;
    I2C_WaitIdle();
}

void I2C_GenerateRestart(void) {
    SSP1CON2bits.RSEN = 1;
    I2C_WaitIdle();
}

void I2C_GenerateSTOP(void) {
    SSP1CON2bits.PEN = 1;
    I2C_WaitIdle();
}