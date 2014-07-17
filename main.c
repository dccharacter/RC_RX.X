/* 
 * File:   main.c
 * Author: dccharacter
 *
 * Created on June 24, 2014, 10:21 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>

#include "hw_conf.h"
#include "usart.h"
#include "utils.h"


__CONFIG(FOSC_INTOSC & CLKOUTEN_OFF & WDTE_OFF & PWRTE_OFF & PLLEN_OFF);

uint16_t tmr1_tmp;

uint16_t chA_dif = 600;
uint16_t chB_dif = 1200;
uint16_t chC_dif = 2200;
uint8_t text_buf[100];
uint16_t adc_res;

typedef struct {
    uint16_t cntr;
    uint16_t impulse_in_us;
} ch_mes;

ch_mes chA, chB, chC;

/*
 * 
 */
void main(void) {
    hw_config();

    serialPutS ("\fReset\r\n");

#ifdef PWM_SERVO
    set_tmr2pwm_dc(chA_dif);
    set_tmr4pwm_dc(chB_dif);
    set_tmr6pwm_dc(chC_dif);
#endif //#ifdef PWM_SERVO
    int8_t diff = 100;

    while(1)
    {
        GO_nDONE = 1;
        while (!GO_nDONE);
        adc_res = (ADRESH << 8) + ADRESL;
        adc_res = (adc_res*TMR_PR_SET/1023);
        CCPR1L = adc_res;
        CCPR2L = adc_res;



#ifdef PWM_SERVO
        if ((chA_dif >= 3000) || (chA_dif <= 500))
        {
            diff = -diff;
        }
        chA_dif += diff;


        set_tmr2pwm_dc(chA_dif);
#endif //#ifdef PWM_SERVO
        
        __delay_ms(500);
        //RB3 = !RB3;
        sprintf(text_buf, "adc: %u, ch_A: %3.0u, ch_B: %3.0u, ch_C: %3.0u,\r\n", adc_res, chA.impulse_in_us, chB.impulse_in_us, chC.impulse_in_us);
        serialPutS (text_buf);
        //sprintf(text_buf, "adc: %u\r\n", adc_res);
        //serialPutS (text_buf);
        //RB3 = !RB3;
        /*if (chA.process_flag)
        {
            chA.process_flag = 0;
        }
        if (chB.process_flag)
        {
            chB.process_flag = 0;
        }
        if (chC.process_flag)
        {
            chC.process_flag = 0;
        }*/

    }
}

void interrupt isr (void)
{
    if (IOCIE && IOCIF)
    {
        tmr1_tmp = (TMR1H << 8) + TMR1L;
        if (IOCBF1)
        {
            if (RB1)
            {
                chA.cntr = tmr1_tmp;
            } else
            {
                chA.impulse_in_us = (tmr1_tmp - chA.cntr)/4;
            }
            IOCBF1 = 0;
        }
        if (IOCBF4)
        {
            if (RB4)
            {
                chB.cntr = tmr1_tmp;
            } else
            {
                chB.impulse_in_us = (tmr1_tmp - chB.cntr)/4;
            }
            IOCBF4 = 0;
        }
        if (IOCBF3)
        {
            if (RB3)
            {
                chC.cntr = tmr1_tmp;
            } else
            {
                chC.impulse_in_us = (tmr1_tmp - chC.cntr)/4;
            }
            IOCBF3 = 0;
        }
    }
}

