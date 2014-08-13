/* 
 * File:   main.h
 * Author: dccharacter
 *
 * Created on July 27, 2014, 9:39 PM
 */

#ifndef MAIN_H
#define	MAIN_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "hw_conf_v1.h"
#include "stdint.h"

#define TIME_BASE_HZ    (50) //Hz
#define UPDATE_DISPLAY_TMR0 (TMR0_ROLLOVER_FREQ/TIME_BASE_HZ)

#define PID_REFRESH_DIV (1)
#define ADC_REFRESH_DIV (5)
#define DISP_REFRESH_DIV (10)

#define PID_KOEFF_P  20
#define PID_KOEFF_I  800
#define PID_KOEFF_D  0

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
#ifdef ENABLE_CH_C
    ch_mes chC = {0, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US, NEUTRAL_PLS_US};
#endif //#ifdef ENABLE_CH_C

    struct {
        uint8_t time_base : 1;
        uint8_t upd_pid : 1;
        uint8_t upd_disp : 1;

    } flags;


#ifdef	__cplusplus
}
#endif

#endif	/* MAIN_H */

