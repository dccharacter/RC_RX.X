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

uint16_t getMaxCCPR (uint16_t bat_vltg);
uint16_t get_Vbat(void);
void set_CCPR(uint16_t vbat, uint16_t control_pulse, uint16_t ail_pulse);
uint16_t normalize_pulse (uint16_t puse, uint16_t max_pulse, uint16_t min_pulse);

uint16_t tmr1_tmp;

uint16_t chA_dif = 600;
uint16_t chB_dif = 1200;
uint16_t chC_dif = 2200;
uint8_t text_buf[100];
uint16_t v_bat;

uint8_t fail_safe_mode = FSM_INIT;

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
    serialPutS("Test");
    while(1)
    {
        v_bat = get_Vbat();
        set_CCPR(v_bat, THROTTLE.impulse_in_us, AILERONE.impulse_in_us);
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
                chC.cntr = tmr1_tmp;
            } else
            {
                chC.impulse_in_us = (tmr1_tmp - chC.cntr)/4;
            }
            IOCBF4 = 0;
        }
        if (IOCBF3)
        {
            if (RB3)
            {
                chB.cntr = tmr1_tmp;
            } else
            {
                chB.impulse_in_us = (tmr1_tmp - chB.cntr)/4;
            }
            IOCBF3 = 0;
        }
    }
}

uint16_t getMaxCCPR (uint16_t bat_vltg)
{
    /* Maximum PWM period is TMR_PR_SET - this corresponds to battery voltage
     * We need to limit the voltage on motors to MAX_MTR_VLTG_MV
     * Thus PWM period will be TMR_PR_SET * MAX_MTR_VLTG_MV / battery_voltage
     */
    return (uint16_t)((TMR_PR_SET * MAX_MTR_VLTG_MV) / bat_vltg);
}

uint16_t get_Vbat(void) {
    GO_nDONE = 1;
    while (!GO_nDONE);
    v_bat = (ADRESH << 8) + ADRESL;
    return (v_bat * 13); // in mV !!! 13 is resistor devider ratio = (4700 + 390) / 390; sinse Vref = 1.024 and ADC is 10 bits, ADC count is the voltage in mV
}

void set_CCPR(uint16_t v_bat, uint16_t thr_pulse, uint16_t ail_pulse) {
    uint16_t pwm_dc_l = 0;
    uint16_t pwm_dc_r = 0;
    uint16_t max_pwm_dc = TMR_PR_SET;

    if (fail_safe_mode != FSM_DIS) {
        switch (fail_safe_mode) {
            case FSM_INIT:
                if (thr_pulse > 1600) {
                    fail_safe_mode = FSM_HIGH_1;
                }
                break;
            case FSM_HIGH_1:
                if (thr_pulse < 1200) {
                    fail_safe_mode = FSM_LOW_1;
                }
                break;
            case FSM_LOW_1:
                if (thr_pulse > 1600) {
                    fail_safe_mode = FSM_HIGH_2;
                }
                break;
            case FSM_HIGH_2:
                if (thr_pulse < 1200) {
                    fail_safe_mode = FSM_DIS;
                }
                break;
        }
    }
    else if (fail_safe_mode == FSM_DIS) {
        if (v_bat > MAX_MTR_VLTG_MV) {
            uint16_t max_pwm_dc = (uint16_t) ((TMR_PR_SET * MAX_MTR_VLTG_MV) / v_bat);
        }

        thr_pulse = normalize_pulse(thr_pulse, MAX_PLS_US, MIN_PLS_US);
        ail_pulse = normalize_pulse(ail_pulse, MAX_PLS_US, MIN_PLS_US);

        uint16_t corr_pls_l = (uint16_t) (thr_pulse - (((thr_pulse - MIN_PLS_US) * (ail_pulse - AIL_MID_POINT)) / 4500l));


        uint16_t corr_pls_r = thr_pulse - (((thr_pulse - MIN_PLS_US) * (AIL_MID_POINT - ail_pulse)) / 4500l);

        if (ail_pulse > AIL_MID_POINT) {
            pwm_dc_l = (uint16_t) ((max_pwm_dc * (corr_pls_l - MIN_PLS_US)) / PLS_TRAVEL);
            pwm_dc_r = (uint16_t) ((max_pwm_dc * (thr_pulse - MIN_PLS_US)) / PLS_TRAVEL);
        } else {
            pwm_dc_l = (uint16_t) ((max_pwm_dc * (thr_pulse - MIN_PLS_US)) / PLS_TRAVEL);
            pwm_dc_r = (uint16_t) ((max_pwm_dc * (corr_pls_r - MIN_PLS_US)) / PLS_TRAVEL);
        }

        //CCP1CONbits.DC1B = pwm_dc_l & 0b11;
        CCPR1L = (uint8_t) pwm_dc_l;
        //CCP2CONbits.DC2B = pwm_dc_r & 0b11;
        CCPR2L = (uint8_t) pwm_dc_r;
    }
    sprintf(text_buf, "mode: %u, thro: %3.0u, CCPRL: %u\r\n", fail_safe_mode, thr_pulse, CCPR1L);
    serialPutS(text_buf);
}

uint16_t normalize_pulse (uint16_t pulse, uint16_t max_pulse, uint16_t min_pulse)
{
    if (pulse > max_pulse && pulse <3000) // fail-safe
    {
        return max_pulse;
    }
    else if (pulse < min_pulse)
    {
        return min_pulse;
    }
    return pulse;
}

