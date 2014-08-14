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
#include "hw_conf_v1.h"


__CONFIG(FOSC_INTOSC & CLKOUTEN_OFF & WDTE_OFF & PWRTE_ON & PLLEN_OFF);

#define AXIS AXIS_Y

uint8_t text_buf[50];

typedef enum {
    FSM_INIT = 0, FSM_HIGH_1, FSM_LOW_1, FSM_HIGH_2, FSM_DIS
} fail_safe_mode;

typedef enum {
    OPM_CAL_MID = 0, OPM_CAL_MAX, OPM_FSM, OPM_FLGHT, OPM_HALT, OPM_INIT, OPM_BAT_LOW
} operation_mode;

fail_safe_mode fsm_mode = FSM_INIT;
operation_mode oper_mode = OPM_INIT;

typedef struct {
    uint32_t ival;
    uint32_t err_filter;
    uint32_t min;
    uint32_t max;
    int32_t err;
    uint32_t out;
    uint32_t ki;
    uint32_t kp;
    uint32_t kd;
} REG_type;
REG_type reg;

uint8_t upd_disp_cntr = 0;
uint8_t upd_adc_cntr = 0;
uint8_t pid_disp_cntr = 0;

uint16_t v_bat_sample;

uint16_t get_Vbat(void);
void apply_controls(uint16_t vbat, ch_mes * strct_thro, ch_mes * strct_ail);
int16_t chop_pulse(int16_t pulse, int16_t min_val, int16_t max_val, int16_t neutral);
void calibrate_neutral(ch_mes * strct_ch);
void calibrate_extremes(ch_mes * strct_ch);
uint8_t ifSignalGood(int16_t signal);
uint8_t ifFSMdisarmed(void);
void blink_long(void);
void blink_short(void);
void PID_Init(REG_type *reg);
uint16_t PID_Step(REG_type *reg_struct, int16_t err);
void timeBaseRoutine(void);

/*
 * 
 */
void main(void) {
    hw_config();
    uint16_t v_bat = 0;
    PID_Init(&reg);
    blink_long();
    serialPutS("\fR\r\n");
    while (1) {
        timeBaseRoutine();

        switch (oper_mode) {
            case OPM_FLGHT:
                if (flags.upd_pid) {
                    flags.upd_pid = 0;
                    apply_controls(v_bat, &THROTTLE, &AILERONE);
                }
                break;
            case OPM_HALT:
                LED_ON;
                IOCIE = 0; //this ideally should cut throttle and set ailerons to neutral;
                break;
            case OPM_INIT:
                blink_short();
                //serialPutS("Changing mode to FSM. Are you feeling safe?\r\n");
                //serialPutS("\tTo disarm FSM, turn THROTTLE stick to MIN\r\n");
                oper_mode = OPM_FSM;
                break;
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
        }

    }
}

void timeBaseRoutine(void) {
    if (!flags.time_base) {
        return;
    }

    flags.time_base = 0;
    if (upd_adc_cntr++ >= ADC_REFRESH_DIV) {
        upd_adc_cntr = 0;
        if (ADIF) {
            ADIF = 0;
            v_bat_sample = (ADRESH << 8) + ADRESL;
        }
        v_bat_sample = get_Vbat();
        GO_nDONE = 1;
    }
    if (upd_disp_cntr++ >= DISP_REFRESH_DIV) {
        //uint16_t axs = ITG3200_ReadGyroAxis(AXIS);
        //sprintf(text_buf, "reg: %i\r\n", axs);
        //serialPutS(text_buf);
        flags.upd_disp = 1;
        upd_disp_cntr = 0;
#ifndef FLIGHT_TX
        CB_BytesInBuf(&usart_buf);
        if (CB_BytesInBuf(&usart_buf) >= 4) {
            uint8_t dt[4];
            dt[0] = CB_ReadFromBuf(&usart_buf);
            dt[1] = CB_ReadFromBuf(&usart_buf);
            dt[2] = CB_ReadFromBuf(&usart_buf);
            dt[3] = CB_ReadFromBuf(&usart_buf);
            sprintf(text_buf, "%c%c%c%c", dt[0], dt[1], dt[2], dt[3]);
            serialPutS(text_buf);
            if (dt[3] == 'p' || dt[3] == 'P') {
                switch (dt[0]) {
                    case 'i':
                        reg.ki = dt[2] + ((uint16_t)dt[1] << 8);
                        break;
                    case 'p':
                        reg.kp = dt[2] + ((uint16_t)dt[1] << 8);
                        break;
                    case 'd':
                        reg.kd = dt[2] + ((uint16_t)dt[1] << 8);
                        break;
                }
            }
        }
#endif
#ifndef FLIGHT_RX
        //sprintf(text_buf, "p %i i %i d %i\r\n", (uint16_t)reg.kp, (uint16_t)reg.ki, (uint16_t)reg.kd);
        //serialPutS(text_buf);
#endif
    }
    if (pid_disp_cntr++ >= PID_REFRESH_DIV) {
        flags.upd_pid = 1;
        pid_disp_cntr = 0;
    }
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
    uint16_t thro_ac = 0;
    uint16_t pwm_dc_l = 0;
    uint16_t pwm_dc_r = 0;
    uint16_t max_CCPRL_fast = TMR_PR_SET << 2; //TMR is 8 bits and PWM is 10 bits, so we have to add 2 bits

    v_bat = 7000;

    if (v_bat > MAX_MTR_VLTG_MV) {
        //serialPutS("test");
        max_CCPRL_fast = (uint16_t) ((max_CCPRL_fast * MAX_MTR_VLTG_MV) / v_bat);
    }

    thro->pulse = chop_pulse(thro->pulse, thro->min_pls, thro->max_pls, thro->neutral);
    if (thro->pulse < thro->min_pls) {
        thro->min_pls = thro->pulse;
    }
    thro_ac = thro->pulse - thro->min_pls;
    pwm_dc_l = (uint16_t) ((thro_ac * (long) max_CCPRL_fast) / (thro->max_pls - thro->min_pls));
    reg.max = pwm_dc_l >> 3;

    int16_t gyro_axis = 0;//ITG3200_ReadGyroAxis(AXIS);
    //ail->pulse = chop_pulse(ail->pulse, ail->min_pls, ail->max_pls, ail->neutral);
    //int16_t ail_correction = ail->neutral - ail->pulse;

    //gyro_axis -= (ail_correction * 6);

    uint16_t corr = PID_Step(&reg, gyro_axis);
    pwm_dc_r = pwm_dc_l - corr;




    //uint16_t max_CCPRL_slow = ((long) max_CCPRL_fast) * FLY_BAR_COMPENSATION / 100;

    //int16_t max_ail_amplitude = thro_ac;
    //int16_t ail_correction = ail->neutral - ail->pulse;
    //ail_correction = (int16_t) ((ail_correction * (long) max_ail_amplitude * AIL_PRCT) / ((ail->neutral - ail->min_pls)*100));


    /*if (ail_correction > 0) {
        pwm_dc_l = thro_ac - ail_correction;
        pwm_dc_r = thro_ac;
    } else {
        pwm_dc_l = thro_ac;
        pwm_dc_r = thro_ac + ail_correction;
    }*/

    //pwm_dc_r = (pwm_dc_r * (long) max_CCPRL_slow) / (thro->max_pls - thro->min_pls);
    // TODO: solve this - why did I have 1023 in there???

    //pwm_dc_l = (pwm_dc_l * (long) max_CCPRL_fast) / 1023;
    //pwm_dc_r = (pwm_dc_r * (long) max_CCPRL_slow) / 1023;

#ifdef ENABLE_MOTORS
    CCP1CONbits.DC1B = (uint8_t) (pwm_dc_l & 0b11);
    CCPR1L = (uint8_t) (pwm_dc_l >> 2);
    CCP2CONbits.DC2B = (uint8_t) (pwm_dc_r & 0b11);
    CCPR2L = (uint8_t) (pwm_dc_r >> 2);
#endif //#ifdef ENABLE_MOTORS

#ifdef UART
    if (flags.upd_disp) {
        flags.upd_disp = 0;
        sprintf(text_buf, "co %u th%i ai%i mpw%u rpw%u lpw%u\r\n", corr, thro->pulse, gyro_axis, max_CCPRL_fast, pwm_dc_r, pwm_dc_l);
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
    ITG3200_InitTypeDef ITG3200_IintStruct;
    ITG3200_IintStruct.ITG3200_DigitalLowPassFilterCfg = ITG3200_LPFBandwidth42Hz;
    ITG3200_IintStruct.ITG3200_IrqConfig = 0;
    ITG3200_IintStruct.ITG3200_PowerControl = 0 | ITG3200_CLKLPLLwX_GyroRef;
    ITG3200_IintStruct.ITG3200_SampleRateDivider = 7;

    if (oper_mode != OPM_FSM) {
        return 0;
    }

    calibrate_neutral(&AILERONE); //while in FAIL-SAFE mode constantly calibrating AILERONE midpoint with running average
    calibrate_extremes(&THROTTLE);

    switch (fsm_mode) {
        case FSM_INIT:
            if (THROTTLE.pulse < 1200) {
                blink_short();
                serialPutS("\tto MAX\r\n");
                fsm_mode = FSM_LOW_1;
            }
            break;
        case FSM_LOW_1:
            if (THROTTLE.pulse > 1600) {
                blink_short();
                serialPutS("Start cal\r\n");
                ITG3200_SetRevPolarity(0, 0, 0);
                ITG3200_Init(&ITG3200_IintStruct);
                serialPutS("Done cal\r\n");
                serialPutS("\tto MIN\r\n CAUTION: FSM will be disarmed !!!!\r\n");
                fsm_mode = FSM_HIGH_2;
            }
            break;
        case FSM_HIGH_2:
            if (THROTTLE.pulse < 1200) {
                serialPutS("\tFSM disarmed\r\n");
                fsm_mode = FSM_DIS;
            }
            break;
    }
    if (fsm_mode == FSM_DIS) {
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

void PID_Init(REG_type *reg) {
    reg->ival = 0;
    reg->err_filter = 0;
    reg->min = 0;
    reg->max = 255;
    reg->ki = PID_KOEFF_I;
    reg->kp = PID_KOEFF_P;
    reg->kd = PID_KOEFF_D;
}

uint16_t PID_Step(REG_type *reg, int16_t err) {
    uint16_t res;
    uint16_t diff;

    reg->err = err;

    //diff = err - (reg->err_filter >> 4); //Input filter+differentiator
    //reg->err_filter += diff;
    diff = err - reg->err_filter; //Input filter+differentiator
#define FLT 8L
    reg->err_filter = (reg->err_filter * (FLT - 1) + err) / FLT;

    reg->ival += ((int32_t) err) * reg->ki;
    res = reg->ival >> 16;

    if (res < reg->min) {
        res = reg->min;
        reg->ival = ((int32_t) res) << 16;
    } else if (res > reg->max) {
        res = reg->max;
        reg->ival = ((int32_t) res) << 16;
    }

    res += (((int32_t) err * reg->kp) >> 8)+(((int32_t) diff * reg->kd) >> 8);

    if (res < reg->min) {
        res = reg->min;
    } else if (res > reg->max) {
        res = reg->max;
    }

    reg->out = res;

    return res;
}
