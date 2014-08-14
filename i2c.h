/* 
 * File:   i2c.h
 * Author: dccharacter
 *
 * Created on August 2, 2014, 10:43 PM
 */

#ifndef I2C_H
#define	I2C_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>

#define  I2C_Direction_Transmitter      ((uint8_t)0x00)
#define  I2C_Direction_Receiver         ((uint8_t)0x01)

#define ACK_STATUS uint8_t
#define ACK     0
#define NACK    1

void I2C_GenerateStart(void);
void I2C_GenerateRestart(void);
void I2C_GenerateSTOP(void);
ACK_STATUS I2C_SendByte (uint8_t Byte);
uint8_t I2C_GetByte(void);
void I2C_MultiWrite(uint8_t * pBuffer);
void I2C_MultiRead(uint8_t* pBuffer, uint16_t NumByteToRead);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

