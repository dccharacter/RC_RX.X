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

void set_tmr2pwm_dc(uint16_t dc_in_us);
void set_tmr4pwm_dc(uint16_t dc_in_us);
void set_tmr6pwm_dc(uint16_t dc_in_us);


#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_H */

