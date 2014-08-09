#include <xc.h>
#include "i2c.h"
#include "main.h"
#include "usart.h"

#define FAILS 20
uint8_t fail_counter = FAILS;

/* Local defenitions */
void I2C_WaitIdle(void);
void I2C_Ack();
void I2C_Nack();

void I2C_GenerateStart(void);
void I2C_GenerateRestart(void);
void I2C_GenerateSTOP(void);
ACK_STATUS I2C_SendByte(uint8_t Byte);
uint8_t I2C_GetByte(void);

uint16_t i2c_to;

void I2C_Ack(void) {
    I2C_WaitIdle();
    SSP1CON2bits.ACKDT = 0;
    SSP1CON2bits.ACKEN = 1;
}

void I2C_Nack(void) {
    I2C_WaitIdle();
    SSP1CON2bits.ACKDT = 1;
    SSP1CON2bits.ACKEN = 1;
}

void I2C_WaitIdle(void) {
    if (SSP1CON1bits.SSPOV) {
        serialPutS("OVERFLOW\r\n");
        while (1);
    }
    while ((SSP1CON2 & 0x1F) || (SSP1STATbits.R_nW));
    /* wait for any pending transfer */
}

uint8_t I2C_GetByte() {
    volatile unsigned char temp;
    I2C_WaitIdle();
    /* Reception works if transfer is initiated in read mode */
    SSP1CON2bits.RCEN = 1; /* Enable data reception */
    I2C_WaitIdle();
    temp = SSP1BUF; /* Read serial buffer and store in temp register */
    I2C_WaitIdle();
    I2C_Nack();
    return temp; /* Return the read data from bus */
}

ACK_STATUS I2C_SendByte(uint8_t Byte) {
    ACK_STATUS result;
    I2C_WaitIdle();
    SSP1BUF = Byte;
    I2C_WaitIdle();
    result = SSP1CON2bits.ACKSTAT;
    return result;
}

void I2C_GenerateStart(void) {
    I2C_WaitIdle();
    SSP1CON2bits.SEN = 1;
}

void I2C_GenerateRestart(void) {
    I2C_WaitIdle();
    SSP1CON2bits.RSEN = 1;
}

void I2C_GenerateSTOP(void) {
    I2C_WaitIdle();
    SSP1CON2bits.PEN = 1;
}

void I2C_Configure(void) {
    SSP1CON1bits.SSPEN = 0;
    __delay_ms(2);
    TRISB1 = 0;
    TRISB4 = 0;
    RB1 = 0;
    RB4 = 0;
    TRISB1 = 1;
    TRISB4 = 1;
    SSP1STATbits.SMP = 1; //0 = Slew rate control enabled for high speed mode (400 kHz)
    SSP1STATbits.CKE = 0; // 1 = Enable input logic so that thresholds are compliant with SM bus™ specification
    SSP1CON1bits.SSPM = 0b1000; // 1000 = I2C Master mode, clock = FOSC/(4 * (SSPxADD+1))
    SSP1ADD = _XTAL_FREQ / (4 * 100000l) - 1; //400kHz
    SSP1CON3bits.BOEN = 1; // Buffer overwrite
    SSP1CON3bits.SDAHT = 0; //1 = Minimum of 300 ns hold time on SDAx after the falling edge of SCLx
    SSP1CON1bits.SSPEN = 1;
}

uint8_t I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t result = 0;
    I2C_GenerateStart();
    result |= I2C_SendByte(addr);
    result |= I2C_SendByte(reg);
    result |= I2C_SendByte(val);
    I2C_GenerateSTOP();
    return result;
}

uint8_t I2C_WriteMultiRegs(uint8_t addr, uint8_t start_reg, uint8_t numbytes, uint8_t *data) {
    uint8_t result = 0;
    I2C_GenerateStart();
    result |= I2C_SendByte(addr);
    result |= I2C_SendByte(start_reg);
    while (numbytes--) {
        result |= I2C_SendByte(*data++);
    }
    I2C_GenerateSTOP();
    return result;
}

uint8_t I2C_ReadReg(uint8_t addr, uint8_t reg) {
    uint8_t result;
    I2C_GenerateStart();
    I2C_SendByte(addr & 0xFE);
    I2C_SendByte(reg);
    I2C_GenerateRestart();
    I2C_SendByte(addr | 1);
    result = I2C_GetByte();
    I2C_GenerateSTOP();
    return result;
}

uint8_t I2C_ReadMultiRegs(uint8_t addr, uint8_t start_reg, uint8_t numbytes, uint8_t *data) {
    I2C_GenerateStart();
    I2C_SendByte(addr & 0xFE);
    I2C_SendByte(start_reg);
    I2C_GenerateRestart();
    I2C_SendByte(addr | 1);
    while (numbytes--) {
        *data++ = I2C_GetByte();
    }
    I2C_GenerateSTOP();
    return 0;
}
