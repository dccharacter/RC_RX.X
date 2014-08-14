#include <stdint.h>
#include "main.h"
#include "utils.h"
#include <xc.h>
#include "hw_conf_v1.h"

void interrupt isr(void) {
    uint16_t tmr1_tmp;
    if (IOCIF) {
        tmr1_tmp = (TMR1H << 8) + TMR1L;
        if (IOCBF3) {
            if (RB3) {
                chB.cntr = tmr1_tmp;
            } else {
                chB.pulse = (int16_t) ((chB.pulse * (CAL_FILTER - 1) + ((tmr1_tmp - chB.cntr) / 4)) / CAL_FILTER);
            }
            IOCBF3 = 0;
        }
        if (IOCBF6) {
            if (RB6) {
                chA.cntr = tmr1_tmp;
            } else {
                chA.pulse = (int16_t) ((tmr1_tmp - chA.cntr) / 4);
            }
            IOCBF6 = 0;
        }
        /*if (IOCBF7) {
            if (RB7) {
                chC.cntr = tmr1_tmp;
            } else {
                chC.pulse = (int16_t) ((tmr1_tmp - chC.cntr) / 4);

            }
            IOCBF7 = 0;
        }*/
    }
    if (TMR0IF) {
        TMR0IF = 0;
        static uint16_t tmr0_cntr = 0;
        if (tmr0_cntr++ >= UPDATE_DISPLAY_TMR0) {
            flags.time_base = 1;
            tmr0_cntr = 0;
        }
    }
#ifndef FLIGHT_TX
    if (RCIF) {
        RCIF = 0;
        CB_PutInBuf(&usart_buf, RCREG);
    }
#endif //#ifndef FLIGHT_TX
}
