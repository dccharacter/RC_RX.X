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

#include "main.h"
#include "usart.h"
#include "utils.h"
#include "ITG3200.h"


__CONFIG(FOSC_INTOSC & CLKOUTEN_OFF & WDTE_OFF & PWRTE_ON & PLLEN_OFF);

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

uint8_t updt_dspl = 0;

ch_mes chA = {0, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US};
ch_mes chB = {0, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, MIN_PLS_US, MAX_PLS_US};
ch_mes chC = {0, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US};

uint16_t v_bat_sample;

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

int16_t vals[4];
/*
 * 
 */
void main(void) {
    uint16_t v_bat;

    hw_config();

    ITG3200_InitTypeDef ITG3200_IintStruct;
    ITG3200_IintStruct.ITG3200_DigitalLowPassFilterCfg = ITG3200_LPFBandwidth42Hz;
    ITG3200_IintStruct.ITG3200_IrqConfig = 0;
    ITG3200_IintStruct.ITG3200_PowerControl = 0 | ITG3200_CLKLPLLwX_GyroRef;
    ITG3200_IintStruct.ITG3200_SampleRateDivider = 7;

    serialPutS("\fReset\r\n");

    ITG3200_Init(&ITG3200_IintStruct);

    uint8_t dev_id;

    while (1) {
        v_bat = get_Vbat();
        if (updt_dspl) {
            ITG3200_GetMeasurements(vals);
            //vals[0] =  ITG3200_ReadDevID();
            sprintf(text_buf, "v %3u, x 0x%04x, y %3d, z %3d\r", v_bat, vals[0], vals[1], vals[2]);
            serialPutS(text_buf);
            vals[0] = 0x3e;
        }
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
                    //serialPutS("Mode has been changed to FLIGHT. Good luck!\r\n");
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
                //serialPutS("Changing mode to FSM. Are you feeling safe?\r\n");
                //serialPutS("\tTo disarm FSM, turn THROTTLE stick to MIN\r\n");
                oper_mode = OPM_FSM;
                break;
        }

    }
}

void interrupt isr(void) {
    uint16_t tmr1_tmp;
    if (IOCIE && IOCIF) {
        tmr1_tmp = (TMR1H << 8) + TMR1L;
        if (IOCBF6) {
            if (RB6) {
                chA.cntr = tmr1_tmp;
            } else {
                chA.pulse = (int16_t) ((tmr1_tmp - chA.cntr) / 4);
            }
            IOCBF6 = 0;
        }
        if (IOCBF3) {
            if (RB3) {
                chB.cntr = tmr1_tmp;
            } else {
                chB.pulse = (int16_t) ((chB.pulse * (CAL_FILTER - 1) + ((tmr1_tmp - chB.cntr) / 4)) / CAL_FILTER);
            }
            IOCBF3 = 0;
        }
        if (IOCBF7) {
            if (RB7) {
                chC.cntr = tmr1_tmp;
            } else {
                chC.pulse = (int16_t) ((tmr1_tmp - chC.cntr) / 4);

            }
            IOCBF7 = 0;
        }
    }
    if (ADIE && ADIF) {
        ADIF = 0;
        v_bat_sample = (ADRESH << 8) + ADRESL;
        GO_nDONE = 1;
    }
    if (TMR0IF && TMR0IE) {
        TMR0IF = 0;
        static uint16_t tmr0_cntr = 0;
        if (tmr0_cntr++ >= UPDATE_DISPLAY_TMR0)
        {
            updt_dspl = 1;
            tmr0_cntr = 0;
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
    /*
    // There's a bug in PIC16F1827 that gets ADC stuck and conversion never completes.
    // That's how you lose your helicopter
    GO_nDONE = 1;
    while (!GO_nDONE);
    v_bat_new = (ADRESH << 8) + ADRESL;
    v_bat_new *= 13;
    return v_bat_new;
     */
    v_bat_old = (uint16_t) ((v_bat_old * (VBAT_FILTER - 1) + (v_bat_sample * 13)) / VBAT_FILTER);
    return v_bat_old; // in mV !!! 13 is resistor devider ratio = (4700 + 390) / 390; sinse Vref = 1.024 and ADC is 10 bits, ADC count is the voltage in mV
}

void apply_controls(uint16_t v_bat, ch_mes * thro, ch_mes * ail) {
    uint16_t max_CCPRL_fast = TMR_PR_SET;

    if (v_bat > MAX_MTR_VLTG_MV) {
        //serialPutS("test");
        max_CCPRL_fast = (uint16_t) ((TMR_PR_SET * MAX_MTR_VLTG_MV) / v_bat);
    }
    uint16_t max_CCPRL_slow = ((long) max_CCPRL_fast) * FLY_BAR_COMPENSATION / 100;

    uint16_t pwm_dc_l = 0;
    uint16_t pwm_dc_r = 0;

    int16_t thro_ac = 0;

    thro->pulse = chop_pulse(thro->pulse, thro->min_pls, thro->max_pls, thro->neutral);
    ail->pulse = chop_pulse(ail->pulse, ail->min_pls, ail->max_pls, ail->neutral);

    if (thro->pulse < thro->min_pls) {
        thro->min_pls = thro->pulse;
    }
    thro_ac = thro->pulse - thro->min_pls;

    int16_t max_ail_amplitude = thro_ac;
    int16_t ail_correction = ail->neutral - ail->pulse;
    ail_correction = (int16_t) ( (ail_correction * (long) max_ail_amplitude * AIL_PRCT) / ((ail->neutral - ail->min_pls)*100));


    //pwm_dc_l = chop_pulse(thro->impulse_in_us + ail_correction, thro->min_pls, thro->max_pls, thro->neutral) - thro->min_pls;
    //pwm_dc_r = chop_pulse(thro->impulse_in_us - ail_correction, thro->min_pls, thro->max_pls, thro->neutral) - thro->min_pls;

    if (ail_correction > 0) {
        pwm_dc_l = thro_ac - ail_correction;
        pwm_dc_r = thro_ac;
    } else {
        pwm_dc_l = thro_ac;
        pwm_dc_r = thro_ac + ail_correction;
    }

    pwm_dc_l = (pwm_dc_l * (long) max_CCPRL_fast) / (thro->max_pls - thro->min_pls);
    pwm_dc_r = (pwm_dc_r * (long) max_CCPRL_slow) / (thro->max_pls - thro->min_pls);
    // TODO: solve this - why did I have 1023 in there???

    //pwm_dc_l = (pwm_dc_l * (long) max_CCPRL_fast) / 1023;
    //pwm_dc_r = (pwm_dc_r * (long) max_CCPRL_slow) / 1023;

#ifdef ENABLE_MOTORS
    CCP1CONbits.DC1B = pwm_dc_l & 0b11;
    CCPR1L = (uint8_t) pwm_dc_l;
    CCP2CONbits.DC2B = pwm_dc_r & 0b11;
    CCPR2L = (uint8_t) pwm_dc_r;
#endif //#ifdef ENABLE_MOTORS

#ifdef UART
    if (updt_dspl) {
        updt_dspl = 0;
        sprintf(text_buf, "th%iail%impw%urpw%ulpw%u\r\n", thro->pulse, ail->pulse, max_CCPRL_fast, pwm_dc_r, pwm_dc_l);
        serialPutS(text_buf);
    }
#endif //#ifdef UART
}

int16_t chop_pulse(int16_t pulse, int16_t min_val, int16_t max_val, int16_t neutral) {
    if (ifSignalGood(pulse)) {
        if (pulse > max_val) // fail-safe
        {
            return max_val;
        } else if (pulse < min_val) {
            return min_val;
        } else {
            return pulse;
        }
    }
    return neutral;
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
                serialPutS("\tAnd now to MAX\r\n");
                fail_safe_mode = FSM_LOW_1;
            }
            break;
        case FSM_LOW_1:
            if (THROTTLE.pulse > 1600) {
                blink_short();
                serialPutS("\tAnd again to MIN\r\n CAUTION: After that FSM will be disarmed !!!!\r\n");
                fail_safe_mode = FSM_HIGH_2;
            }
            break;
        case FSM_HIGH_2:
            if (THROTTLE.pulse < 1200) {
                serialPutS("\tFSM disarmed\r\n");
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
#ifndef OLS
    LED = 1;
    __delay_ms(1000);
    LED = 0;
#endif
}

void blink_short(void) {
#ifndef OLS
    LED = 1;
    __delay_ms(300);
    LED = 0;
#endif
}
