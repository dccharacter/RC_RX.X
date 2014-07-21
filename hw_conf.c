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

    while (!OSCSTATbits.HFIOFS); //wait for HFIO

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
    
    /* FVR config
     */
    FVRCONbits.CDAFVR = 0b01; // 01 = 1.024V, 10 = 2.048
    FVRCONbits.ADFVR = 0b01; // 1.024V
    FVRCONbits.FVREN = 1;
    while (!FVRCONbits.FVRRDY); // Wait till FVR is ready to use

    /* Comparator config
     * Negative input is signal
     * Positive input is Vref = 1.024V
     * With shunt resistors = 0.2 Ohm, shutdown will occure when current exceeds 5A
     * Inputs on RA2 and RA3
     * RA2 - channel PX, C12IN2-
     * RA3 - channel PX, C12IN3-
     */
    CM1CON1bits.C1PCH = 0b10; // Connected to FVR
    CM1CON1bits.C1NCH = 0b10; // Connected to C12IN2-, i.e. RA2
    CM1CON0bits.C1POL = 1; // 1 = inverted polarity
    CM1CON0bits.C1ON = 1;

    CM2CON1bits.C2PCH = 0b10; // Connected to FVR
    CM2CON1bits.C2NCH = 0b11; // Connected to C12IN3-, i.e. RA3
    CM2CON0bits.C2POL = 1; // 1 = inverted polarity
    CM2CON0bits.C2ON = 1;


    /* PWM config
     * Two channels ECCP, each channel is driven with two pins (e.g. P1A-P1B and P2A-P2B
     * A and B work in the same phase, just to multiply current that builds boltage on FET gate
     * Auto-shutdown is enabled, driven by comparator
     * Either comparator event would disable both P1 and P2 channels
     * No auto-resume is set
     */
   
    CCP1CON = 0b10001101;
    CCP2CON = 0b10001101;

    CCP1ASbits.CCP1AS = 0;//0b011; //eithe comparator 1 or 2 is high
    CCP1ASbits.PSS1AC = 0;
    CCP1ASbits.PSS1BD = 0;
    PWM1CONbits.P1RSEN = 1; // 0 - no autorestart
    CCP2ASbits.CCP2AS = 0;//0b011; //eithe comparator 1 or 2 is high
    CCP2ASbits.PSS2AC = 0;
    CCP2ASbits.PSS2BD = 0;
    PWM2CONbits.P2RSEN = 1; // 0 - no autorestart

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

    ANSA4 = 1;
    ADCON1bits.ADFM = 1; // 1 = right justified
    ADCON1bits.ADCS = 1; // 1 = Fosc/8
    ADCON1bits.ADNREF = 0; // VSS
    ADCON1bits.ADPREF = 0b11; // FVR
    ADCON0 =  0b00010011;
}
