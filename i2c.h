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

    uint8_t I2C_WriteReg(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t I2C_WriteMultiRegs(uint8_t addr, uint8_t start_reg, uint8_t numbytes, uint8_t *data);
    uint8_t I2C_ReadMultiRegs(uint8_t addr, uint8_t start_reg, uint8_t numbytes, uint8_t *data);
    uint8_t I2C_ReadReg(uint8_t addr, uint8_t reg);
    void I2C_Configure(void);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

