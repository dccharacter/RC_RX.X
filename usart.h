/* 
 * File:   usart.h
 * Author: dccharacter
 *
 * Created on June 24, 2014, 11:33 PM
 */

#ifndef USART_H
#define	USART_H

void serialPutCh (char txByte);
char serialGetCh (void);
void serialPutS (const char * txstr);

#endif	/* USART_H */

