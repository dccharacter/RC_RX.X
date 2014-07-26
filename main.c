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
#include <math.h>

#include "hw_conf.h"
#include "usart.h"
#include "utils.h"


__CONFIG(FOSC_INTOSC & CLKOUTEN_OFF & WDTE_OFF & PWRTE_OFF & PLLEN_OFF);

uint8_t text_buf[100];

uint8_t fail_safe_mode = FSM_INIT;
uint8_t oper_mode = OPM_INIT;

/* All pulses are in us
 *
 */
typedef struct {
    uint16_t cntr;
    int16_t pulse;
    int16_t prev_pulse;
    int16_t neutral;
    int16_t min_pls;
    int16_t max_pls;
} ch_mes;

ch_mes chA = {0, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US};
ch_mes chB = {0, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, MIN_PLS_US, MAX_PLS_US};
ch_mes chC = {0, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US};

uint16_t getMaxCCPR(uint16_t bat_vltg);
uint16_t get_Vbat(void);
void apply_controls(uint16_t vbat, ch_mes * strct_thro, ch_mes * strct_ail);
int16_t chop_pulse(int16_t pulse, int16_t min_val, int16_t max_val, int16_t neutral);
void calibrate_neutral(ch_mes * strct_ch);
void calibrate_extremes(ch_mes * strct_ch);
uint8_t ifSignalGood(int16_t signal);
uint8_t ifFSMdisarmed(void);
void blink_long(void);
void blink_short(void);

/*
 * 
 */
void main(void) {
    uint16_t v_bat;

    hw_config();
    blink_long();
    blink_long();
#ifdef UART
    serialPutS("\fHello!\r\nInitializing. Mode has been changed to INIT\r\n");
#endif //#ifdef UART
    while (1) {
        v_bat = get_Vbat();
        switch (oper_mode) {
            case OPM_CAL_MID:
                calibrate_neutral(&AILERONE);
                break;
            case OPM_CAL_MAX:
                calibrate_extremes(&THROTTLE);
                calibrate_extremes(&AILERONE);
                break;
            case OPM_FSM:
                if (ifFSMdisarmed()) {
#ifdef UART
                    serialPutS("Mode has been changed to FLIGHT. Good luck!\r\n");
#endif //#ifdef UART
                    blink_long();
                    blink_long();
                    oper_mode = OPM_FLGHT;
                }
                break;
            case OPM_FLGHT:
                apply_controls(v_bat, &THROTTLE, &AILERONE);
                break;
            case OPM_HALT:
                while (1);
                break;
            case OPM_INIT:
                blink_short();
#ifdef UART
                serialPutS("Changing mode to FSM. Are you feeling safe?\r\n");
                serialPutS("\tTo disarm FSM, turn THROTTLE stick to MIN\r\n");
#endif //#ifdef UART
                oper_mode = OPM_FSM;
                break;
        }

    }
}

void interrupt isr(void) {
    uint16_t tmr1_tmp;
    if (IOCIE && IOCIF) {
        tmr1_tmp = (TMR1H << 8) + TMR1L;
        if (IOCBF1) {
            if (RB1) {
                chA.cntr = tmr1_tmp;
            } else {
                chA.pulse = (int16_t) ((tmr1_tmp - chA.cntr) / 4);
            }
            IOCBF1 = 0;
        }
        if (IOCBF3) {
            if (RB3) {
                chB.cntr = tmr1_tmp;
            } else {
                chB.pulse = (int16_t) ((chB.pulse * (CAL_FILTER - 1) + ((tmr1_tmp - chB.cntr) / 4)) / CAL_FILTER);
            }
            IOCBF3 = 0;
        }
        if (IOCBF4) {
            if (RB4) {
                chC.cntr = tmr1_tmp;
            } else {
                chC.pulse = (int16_t) ((tmr1_tmp - chC.cntr) / 4);

            }
            IOCBF4 = 0;
        }

    }
}

uint16_t getMaxCCPR(uint16_t bat_vltg) {
    /* Maximum PWM period is TMR_PR_SET - this corresponds to battery voltage
     * We need to limit the voltage on motors to MAX_MTR_VLTG_MV
     * Thus PWM period will be TMR_PR_SET * MAX_MTR_VLTG_MV / battery_voltage
     */
    return (uint16_t) ((TMR_PR_SET * MAX_MTR_VLTG_MV) / bat_vltg);
}

uint16_t get_Vbat(void) {
    static uint16_t v_bat_old;
    uint16_t v_bat_new;
    GO_nDONE = 1;
    while (!GO_nDONE);
    v_bat_new = (ADRESH << 8) + ADRESL;
    v_bat_new *= 13;
    v_bat_old = (uint16_t) ((v_bat_old * (VBAT_FILTER - 1) + v_bat_new) / VBAT_FILTER);
    return v_bat_old; // in mV !!! 13 is resistor devider ratio = (4700 + 390) / 390; sinse Vref = 1.024 and ADC is 10 bits, ADC count is the voltage in mV
}

void apply_controls(uint16_t v_bat, ch_mes * thro, ch_mes * ail) {
    uint16_t max_CCPRL_fast = TMR_PR_SET;

    if (v_bat > MAX_MTR_VLTG_MV) {
        //serialPutS("test");
        max_CCPRL_fast = (uint16_t) ((TMR_PR_SET * MAX_MTR_VLTG_MV) / v_bat);
    }
    uint16_t max_CCPRL_slow = (long) max_CCPRL_fast * FLY_BAR_COMPENSATION / 100;

    uint16_t pwm_dc_l = 0;
    uint16_t pwm_dc_r = 0;

    int16_t thro_ac = 0;

    thro->pulse = chop_pulse(thro->pulse, thro->min_pls, thro->max_pls, thro->neutral);
    ail->pulse = chop_pulse(ail->pulse, ail->min_pls, ail->max_pls, ail->neutral);

    thro_ac = thro->pulse - thro->min_pls;
    if (thro_ac < 0) {
        thro_ac = 0;
    }
    int16_t max_ail_amplitude = thro_ac / 2; // divide by two coz we only need half range of the throttle stick
    int16_t ail_correction = ail->neutral - ail->pulse;
    ail_correction = (int16_t) ((ail_correction * (long) max_ail_amplitude / (ail->neutral - ail->min_pls)) * AIL_PRCT / 100);


    //pwm_dc_l = chop_pulse(thro->impulse_in_us + ail_correction, thro->min_pls, thro->max_pls, thro->neutral) - thro->min_pls;
    //pwm_dc_r = chop_pulse(thro->impulse_in_us - ail_correction, thro->min_pls, thro->max_pls, thro->neutral) - thro->min_pls;

    if (ail_correction > 0) {
        pwm_dc_l = thro_ac - ail_correction;
        pwm_dc_r = thro_ac;
    } else {
        pwm_dc_l = thro_ac;
        pwm_dc_r = thro_ac + ail_correction;
    }

    pwm_dc_l = (pwm_dc_l * (long) max_CCPRL_fast) / 1023;
    pwm_dc_r = (pwm_dc_r * (long) max_CCPRL_slow) / 1023;

#ifdef ENABLE_MOTORS
    CCP1CONbits.DC1B = pwm_dc_l & 0b11;
    CCPR1L = (uint8_t) pwm_dc_l;
    CCP2CONbits.DC2B = pwm_dc_r & 0b11;
    CCPR2L = (uint8_t) pwm_dc_r;
#endif //#ifdef ENABLE_MOTORS

#ifdef UART
    sprintf(text_buf, "b: %u, th: %i, ac: %i, c1: %u, c2: %u\r\n", v_bat, ail->pulse, max_CCPRL_slow, pwm_dc_l, pwm_dc_r);
    serialPutS(text_buf);
#endif //#ifdef UART
}

int16_t chop_pulse(int16_t pulse, int16_t min_val, int16_t max_val, int16_t neutral) {
    if (ifSignalGood(pulse)) {
        if (pulse > max_val) // fail-safe
        {
            return max_val;
        } else if (pulse < min_val) {
            return min_val;
        }
    } else {
        return neutral;
    }
}

void calibrate_neutral(ch_mes * strct_ch) {
    strct_ch->neutral = strct_ch->pulse;
}

void calibrate_extremes(ch_mes * strct_ch) {
    if (ifSignalGood(strct_ch->pulse)) {
        if (strct_ch->pulse < strct_ch->min_pls) {
            strct_ch->min_pls = strct_ch->pulse;
        } else if (strct_ch->pulse > strct_ch->max_pls) {
            strct_ch->max_pls = strct_ch->pulse;
        }
    }
}

uint8_t ifSignalGood(int16_t signal) {
    if (signal > 800 && signal < 2400) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t ifFSMdisarmed(void) {
    if (oper_mode != OPM_FSM) {
        return 0;
    }

    calibrate_neutral(&AILERONE); //while in FAIL-SAFE mode constantly calibrating AILERONE midpoint with running average
    calibrate_extremes(&THROTTLE);

    switch (fail_safe_mode) {
            /*case FSM_INIT:
                if (THROTTLE.impulse_in_us > 1600) {
                    serialPutS("\tNow turn THROTTLE stick to min\r\n");
                    fail_safe_mode = FSM_HIGH_1;
                }
                break;*/
        case FSM_INIT:
            if (THROTTLE.pulse < 1200) {
                blink_short();
#ifdef UART
                serialPutS("\tAnd now to MAX\r\n");
#endif //#ifdef UART
                fail_safe_mode = FSM_LOW_1;
            }
            break;
        case FSM_LOW_1:
            if (THROTTLE.pulse > 1600) {
                blink_short();
#ifdef UART
                serialPutS("\tAnd again to MIN\r\n CAUTION: After that FSM will be disarmed !!!!\r\n");
#endif //#ifdef UART
                fail_safe_mode = FSM_HIGH_2;
            }
            break;
        case FSM_HIGH_2:
            if (THROTTLE.pulse < 1200) {
#ifdef UART
                serialPutS("\tFSM disarmed\r\n");
#endif //#ifdef UART
                fail_safe_mode = FSM_DIS;
            }
            break;
    }
    if (fail_safe_mode == FSM_DIS) {
        return 1;
    }
    return 0;
}

void blink_long(void) {
    LED = 1;
    __delay_ms(1000);
    LED = 0;
    __delay_ms(500);
}

void blink_short(void) {
    LED = 1;
    __delay_ms(300);
    LED = 0;
    __delay_ms(500);
}
