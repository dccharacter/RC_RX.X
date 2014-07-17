#include <xc.h>
#include <stdint.h>
#include "hw_conf.h"
#include "usart.h"

void hw_config(void)
{
    OSCCONbits.SPLLEN = 0;
    OSCCONbits.IRCF = OSC_SET;
    OSCCONbits.SCS = 0b10;

    OPTION_REGbits.nWPUEN = 0;

    OPTION_REGbits.T0CS = 0; // Fosc/4 clock
    OPTION_REGbits.PSA = 1; // PRESC is not assigned to TMR0, so it's 1:1
    //OPTION_REGbits.PS = 0b000; //1:2 PRESC

    //Pins setup
    APFCON0 = 0b00011001;
    APFCON1 = 0b0;

    ANSELA = 0;
    ANSELB = 0;

    TRISB = 0b11111011;
    TRISA = 0b11111111;

    WPUB = 0b00011010; //RB1,3,4

    //USART setup

    SPBRGH = SPBRGH_CALC;//0;
    SPBRGL = SPBRGL_CALC;//16;
    BRGH = 1;
    BRG16 = 1;
    SYNC = 0;
    TXEN = 1;
    CREN = 1;
    SPEN = 1;
    //RCIE = 1;
    RCIF = 0;
   
    CCP1CON = 0b10001101;
    CCP2CON = 0b10001101;


#ifdef PWM_SERVO
    PR2 = 156; //CCP2
    PR4 = 156; //CCP3
    PR6 = 156; //CCP4

    CCPR2L = 0;
    CCPR3L = 30;
    CCPR4L = 40;

    CCPTMRS = 0b10010000;

    T2CON = 0b00000111;
    T4CON = 0b00000111;
    T6CON = 0b00000111;
#endif //#ifdef PWM_SERVO

#ifdef PWM_DRV_CTRL
    PR2 = TMR_PR_SET;
    PR4 = TMR_PR_SET;

    CCPR1L = 0;
    CCPR2L = 0;

    CCPTMRSbits.C1TSEL = 0; // CCP1 is on TMR2
    CCPTMRSbits.C2TSEL = 1; // CCP2 is on TMR4

    T2CONbits.T2CKPS = TMR_PRESC_SET;
    T4CONbits.T4CKPS = TMR_PRESC_SET;

    TMR2ON = 1;
    TMR4ON = 1;

#endif //#ifdef PWM_DRV_CTRL

    TRISB0 = 0; //P1A
    TRISB5 = 0; //P2B
    
    TRISA7 = 0; //P2A
    TRISA6 = 0; //P2B

    /* RB1 -
     * RB3 -
     * RB4 -
     */
    IOCBP = 0b00011010;
    IOCBN = 0b00011010;
    IOCIE = 1;
    PEIE = 1;
    GIE = 1;


    T1CON = 0b00000001;

    ANSA2 = 1;
    ADCON1bits.ADFM = 1; // 1 = right justified
    ADCON1bits.ADCS = 1; // 1 = Fosc/8
    ADCON1bits.ADNREF = 0; // VSS
    ADCON1bits.ADPREF = 0; // VDD
    ADCON0 =  0b00001011;
}
