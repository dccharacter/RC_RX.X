/* 
 * File:   hw_conf.h
 * Author: dccharacter
 *
 * Created on June 24, 2014, 10:22 PM
 */

#ifndef HW_CONF_H
#define	HW_CONF_H

#include <xc.h>

#define PWM_DRV_CTRL
    #ifndef PWM_DRV_CTRL
        #define PWM_SERVO
    #endif //#define PWM_SERVO

#define OSC_16MHz 0b1111
#define OSC_8MHz 0b1110
#define OSC_4MHz 0b1101
#define OSC_2MHz 0b1100
#define OSC_1MHz 0b1011

#define ADC_Fosc_2  0
#define ADC_Fosc_8  1
#define ADC_Fosc_32 2
#define ADC_Fosc_4  4
#define ADC_Fosc_16 5
#define ADC_Fosc_64 6


#define _XTAL_FREQ 16000000L
#define OSC_SET OSC_16MHz
#define ADC_CLK ADC_Fosc_64
#define PWM_FREQ 30000L
/* Defining TMR2-4-6 Parameters to drive PWM with required frequency */

#define TMR_PRESC ((_XTAL_FREQ/(1024*PWM_FREQ)) + 1)
#define TMR_PRESC_SET 0b00

#if (TMR_PRESC>1)
    #if (TMR_PRESC<4)
        #define TMR_PRESC 4
        #define TMR_PRESC_SET 0b01
    #elif (TMR_PRESC<16)
        #define TMR_PRESC 16
        #define TMR_PRESC_SET 0b10
    #elif (TMR_PRESC>64)
        #error "Prescaler is out of range; Either lower Fosc or rise PWM frequency"
    #else
        #define TMR_PRESC 64
        #define TMR_PRESC_SET 0b11
    #endif
#endif

#define TMR0_FREQ (_XTAL_FREQ/4l) //Hz
#define TMR0_ROLLOVER_FREQ   (TMR0_FREQ/256l) //Hz
#define DSPL_UPDATE_FREQ    (2) //Hz
#define UPDATE_DISPLAY_TMR0 (TMR0_ROLLOVER_FREQ/DSPL_UPDATE_FREQ)


#define TMR_PR_SET (_XTAL_FREQ/(4*PWM_FREQ*TMR_PRESC))


#define XTAL_FREQ_MHZ (_XTAL_FREQ/1000000L)
#define TMR0_CONST (4/XTAL_FREQ_MHZ)

#define I2C_TIMEOUT 1000

/* Defining TMR1 parameters for it to overflow on 2.5 ms
 * TMR1 is driven with Fosc/4 (for CCP purposes we cannot run on Fosc, only Fosc/4)
 * 1 timer tick is 4/Fosc
 * 16 bit timer will overflow in 4*65535/Fosc
 * for 16Mhz it will be ~16ms, i.e. totally enough
 * Fosc     |   Overflow time
 * 1Mhz     |   262ms
 * 2MHz     |   131ms
 * 4MHz     |   64ms
 * 8MHz     |   32ms
 * 16MHz    |   16ms
 * 32MHz    |   8ms
 * So, to measure 0.8ms pulse, prescaler must always be 1:1
 */


#define USART_BAUDRATE 115200L
#define SPBRG_CALC (((_XTAL_FREQ/USART_BAUDRATE)/4)-1)
#define SPBRGH_CALC ((uint8_t)(SPBRG_CALC >> 8))
#define SPBRGL_CALC ((uint8_t)(SPBRG_CALC))

#define MAX_MTR_VLTG_MV 6000l

#define MIN_PLS_US 1100l
#define MAX_PLS_US 1900l
#define NEUTRAL_PLS_US 1500l


/* Fly bar compensation. The top rotor is heavier, so it needs to be slowed down
 * for lower rotor to be able to compensate the momentum.
 * In persent (0-100)
 */
#define FLY_BAR_COMPENSATION 88

/* Limiting heli rotation speed
 * Percent of ailerone signal accounted for
 */
#define AIL_PRCT 3


#define THROTTLE chA
#define AILERONE chB

#define FSM_INIT    0
#define FSM_HIGH_1  1
#define FSM_LOW_1   2
#define FSM_HIGH_2  3
#define FSM_DIS     4

#define OPM_CAL_MID 0
#define OPM_CAL_MAX 1
#define OPM_FSM     2
#define OPM_FLGHT   3
#define OPM_HALT    4
#define OPM_INIT    5
#define OPM_BAT_LOW 6

#define BAT_LOW_VOLTAGE 7400
#define BAT_EXTREME_VOLTAGE 7000

/* Running average filter settings, num of samples
 */
#define CAL_FILTER 8l
#define VBAT_FILTER 8l

#define LED RA0
#define LED_ON (LED=1)
#define LED_OFF (LED=0)

void hw_config(void);

#endif	/* HW_CONF_H */

