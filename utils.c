#include <xc.h>
#include <stdint.h>
#include "utils.h"
#include "hw_conf.h"

#define GET_CCPRxL(pulse_width_in_us) ((((_XTAL_FREQ/TMR_PRESC)*pulse_width_in_us)/1000000L) >> 2)
#define GET_CCPxCON54(pulse_width_in_us) ((((_XTAL_FREQ/TMR_PRESC)*pulse_width_in_us)/1000000L) & 0b11 )

void set_tmr2pwm_dc(uint16_t dc_in_us)
{
    CCPR2L = GET_CCPRxL(dc_in_us);
    CCP2CONbits.DC2B = GET_CCPxCON54(dc_in_us);
}

void set_tmr4pwm_dc(uint16_t dc_in_us)
{
    CCPR3L = GET_CCPRxL(dc_in_us);
    CCP3CONbits.DC3B = GET_CCPxCON54(dc_in_us);
}

void set_tmr6pwm_dc(uint16_t dc_in_us)
{
    CCPR4L = GET_CCPRxL(dc_in_us);
    CCP4CONbits.DC4B = GET_CCPxCON54(dc_in_us);
}

