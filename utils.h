/* 
 * File:   utils.h
 * Author: dccharacter
 *
 * Created on June 25, 2014, 10:54 PM
 */

#ifndef UTILS_H
#define	UTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "hw_conf_v1.h"
#ifndef FLIGHT_TX

#define CB_BUF_SIZE 8

typedef struct {
    uint8_t buf[CB_BUF_SIZE];
    uint8_t idx_in;
    uint8_t idx_out;
} CB_STRUCT;

CB_STRUCT usart_buf;

uint8_t CB_IsEmpty(CB_STRUCT *cb_str);
void CB_PutInBuf(CB_STRUCT *cb_str, uint8_t val);
uint8_t  CB_ReadFromBuf(CB_STRUCT *cb_str);
uint8_t CB_BytesInBuf(CB_STRUCT *cb_str);

#endif //#ifdef FLIGHT_TX

#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_H */

