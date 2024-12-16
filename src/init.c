/*
 *  init.c
 *
 *  Created on: 11.04.2022
 *  Author: Fabian Kurth
 *  Description: initialization for different modules used
 */

#include "DSP28x_Project.h"
#include "init.h"
#include "functions.h"


float PWM_BASE_FREQ = 150000.0;   //10000 kHz
extern float f_TBCLK = 45000000.0;      //45 MHz
float PWM_BASE_DUTY = 0.15;      //50%
float SYS_SAMPLE_TIME_US = 40000;

unsigned long int PWM_TBPRD = 0;
unsigned long int PWM_TBPRD_2HZ = 0;
unsigned long int PWM_CMP = 0;

void SysCtrl(void)
{
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);
    EALLOW;
    DisableDog();
    InitSysCtrl();                                      //setzt u.A. die Frequnez auf 90Mhz
    InitFlash();                                        //Abarbeitung im RAM, nicht FLASH
    InitPieCtrl();                                      //Grundinitialisierung PIE-Gruppe
    InitPieVectTable();                                 //Aufbau Vektortabelle mit Basisfunktionen
    InitCpuTimers();                                    //Betriebsart Rückwärtszähler
    InitAdc();                                          //Initialisierung ADC
    ConfigCpuTimer(&CpuTimer1, 90, SYS_SAMPLE_TIME_US); //Initialisierung Timer 1

    SysCtrlRegs.LOSPCP.bit.LSPCLK = 1;                  //LSPCLK = SYSCLKOUT/2
    CpuTimer1Regs.TCR.bit.TSS = 0;                      //Timer Start
    SysCtrlRegs.LPMCR0.bit.LPM = 0;                     //0 = IDLE-Mode
    EDIS;
}

void ConfigInterrupt(void)
{
    EALLOW;
    /*  Interrupts  */

    PieVectTable.TINT1 =& Timer1_ISR_Sys_control;
    PieVectTable.XINT1 =& SHUTDOWN_ISR;    //Flyback disable
    PieVectTable.XINT2 =& SHUTDOWN_ISR;    //Flyback disable
    PieVectTable.SCIRXINTB =& SCIB_receive_ISR;
    PieVectTable.SCITXINTB =& SCIB_transmit_ISR;
    PieVectTable.ADCINT1 =& CURRENT_ISR;

    IER |= 1;   //XINT1
    IER |= 1<<8;   //Freigabe INT9 (SCIRXINTB)
    IER |= 1<<12;  //Freigabe INT13 (Timer1, nicht durch PIE geschleift)



    PieCtrlRegs.PIEIER1.bit.INTx4 = 1;      //Freigabe XInt1
    PieCtrlRegs.PIEIER1.bit.INTx5 = 1;      //Freigabe XInt2
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;      //Freigabe ADCInt1
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;      //Freigabe SCIRXINTB
    PieCtrlRegs.PIEIER9.bit.INTx4 = 1;      //Freigabe SCITXINTB
    //PieCtrlRegs.PIEIER10.bit.INTx1 = 1;     //Freigabe ADCInt1 Nr 2?????

    GpioIntRegs.GPIOXINT1SEL.bit.GPIOSEL = 8;   //falling edge GPIO8 als XINT1 Interrupt (Shutdown)
    GpioIntRegs.GPIOXINT2SEL.bit.GPIOSEL = 9;   //falling edge GPIO9 als XINT1 Interrupt (Shutdown)
    XIntruptRegs.XINT1CR.bit.ENABLE = 1;        //Int enable XINT1
    XIntruptRegs.XINT1CR.bit.POLARITY = 2;      //falling edge XINT1
    XIntruptRegs.XINT2CR.bit.ENABLE = 1;        //Int enable XINT2
    XIntruptRegs.XINT2CR.bit.POLARITY = 2;      //falling edge XINT2

    EDIS;
}

void ConfigGpio(void)
{
    EALLOW;
    /*  PIN Belegung    */
    //  PWM
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     //ePWM1A = S1
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     //ePWM1B = Temperatur
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     //ePWM2A = Sekundärspannung
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;     //ePWM3A = Überspannung primär
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;     //ePWM1A = Überstrom


    //  GPIO
    //  OUTPUTS
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;    //GPIO17 = P2
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;     //GPIO17 = output
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;    //GPIO44 = P1
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;     //GPIO44 = output
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 0;    //GPIO50 = P3
    GpioCtrlRegs.GPBDIR.bit.GPIO50 = 1;     //GPIO50 = output
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;    //GPIO51 = P4
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;     //GPIO51 = output
    GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;    //GPIO52 = P5
    GpioCtrlRegs.GPBDIR.bit.GPIO52 = 1;     //GPIO52 = output
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;    //GPIO53 = P6
    GpioCtrlRegs.GPBDIR.bit.GPIO53 = 1;     //GPIO53 = output
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;     //GPIO3  = Flyback off
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      //GPIO3  = output

    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;     //GPIO3  = Überstrom erkannt
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;      //GPIO3  = output
    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;      //GPIO3  = off

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;    // Test für ADC
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;

    //  INPUTS
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;     //GPIO7 = Taster S2
    GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;      //GPIO7 = input
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;     //GPIO8 = Taster S1
    GpioCtrlRegs.GPADIR.bit.GPIO8 = 0;      //GPIO8 = input
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;     //GPIO9 = Taster S1
    GpioCtrlRegs.GPADIR.bit.GPIO9 = 0;      //GPIO9 = input
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;    //GPIO10 = Taster SW1
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;     //GPIO10 = input

    //  I2C, SPI
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;    //SDAA = SDA
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;    //SCLA = SCL
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;    //SCITXDB = SCITXDB
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;    //SCIRXDB = SCIRXDB

    EDIS;
}


void ConfigPwm(void)
{
    /*  Konfigurieren der PWM Module    */

    EALLOW;
    PWM_TBPRD_2HZ = (long int)(f_TBCLK / 1280 * 0.5) - 1;
    PWM_TBPRD = (long int)((f_TBCLK * (1 / PWM_BASE_FREQ)) - 1);   //Berechnung des Initial-TBPRD-Wertes zur Festlegung der Frequenz
    PWM_CMP = (int)(PWM_TBPRD * (1 - PWM_BASE_DUTY));       //Berechnung des Initial-CMPA/B-Wertes zur Festlegung des Duty Cycle

    /*  PWM 1   */
    EPwm1Regs.TBPRD = PWM_TBPRD; //100us base PWM Frequency
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;  //Up-Down-Count-Mode
    EPwm1Regs.AQCTLA.bit.ZRO = 0; //deaktiviert
    EPwm1Regs.AQCTLA.bit.PRD = 0; //deaktiviert
    EPwm1Regs.ETSEL.bit.SOCAEN = 1; //ADC-SOC aktiviert
    EPwm1Regs.ETSEL.bit.SOCASEL = 5;    //SOCA wenn CMPA erreicht (falling edge)
    EPwm1Regs.ETPS.bit.SOCAPRD = 1; //SOCA Pulse jedes 1. Event

    //EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD = 1; //clear ePWM1A low wenn Zähler CMPA erreicht (bei Zählrichtung abwärts)
    EPwm1Regs.AQCTLA.bit.CAU = 1; //set ePWM1A high wenn Zähler CMPA erreicht (bei Zählrichtung aufwärts)
    EPwm1Regs.CMPA.half.CMPA = PWM_CMP; //Pulsbreite
    //EPWM1B
    EPwm1Regs.AQCTLB.bit.CBD = 1; //clear ePWM1A low wenn Zähler CMPA erreicht (bei Zählrichtung abwärts)
    EPwm1Regs.AQCTLB.bit.CBU = 2; //set ePWM1A high wenn Zähler CMPA erreicht (bei Zählrichtung aufwärts)
    EPwm1Regs.CMPB = PWM_TBPRD + 1; //Pulsbreite

    /*  PWM 2   */
    EPwm2Regs.TBPRD = PWM_TBPRD_2HZ; //2Hz base PWM Frequency
    EPwm2Regs.TBCTL.bit.CLKDIV = 7; // Faktor 128
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 5;  // Faktor 10
    EPwm2Regs.TBCTL.bit.CTRMODE = 2;  //Up-Down-Count-Mode

    //EPWM2A
    EPwm2Regs.AQCTLA.bit.CAD = 1; //clear ePWM1A low wenn Zähler CMPA erreicht (bei Zählrichtung abwärts)
    EPwm2Regs.AQCTLA.bit.CAU = 2; //set ePWM1A high wenn Zähler CMPA erreicht (bei Zählrichtung aufwärts)
    EPwm2Regs.CMPA.half.CMPA = PWM_TBPRD_2HZ + 1; //Pulsbreite

    /*  PWM 3   */
    EPwm3Regs.TBPRD = PWM_TBPRD_2HZ; //100us base PWM Frequency
    EPwm3Regs.TBCTL.bit.CLKDIV = 7; // Faktor 128
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 5;  // Faktor 10
    EPwm3Regs.TBCTL.bit.CTRMODE = 2;  //Up-Down-Count-Mode

    //EPWM3A
    EPwm3Regs.AQCTLA.bit.CAD = 1; //clear ePWM1A low wenn Zähler CMPA erreicht (bei Zählrichtung abwärts)
    EPwm3Regs.AQCTLA.bit.CAU = 2; //set ePWM1A high wenn Zähler CMPA erreicht (bei Zählrichtung aufwärts)
    EPwm3Regs.CMPA.half.CMPA = PWM_TBPRD_2HZ + 1; //Pulsbreite

    //EPWM3B
    EPwm3Regs.AQCTLB.bit.CBD = 1; //clear ePWM3B low wenn Zähler CMPB erreicht (bei Zählrichtung abwärts)
    EPwm3Regs.AQCTLB.bit.CBU = 2; //set ePWM3B high wenn Zähler CMPB erreicht (bei Zählrichtung aufwärts)
    EPwm3Regs.CMPB = PWM_TBPRD_2HZ + 1; //Pulsbreite

    EDIS;
}

void ConfigAdc(void)
{
    EALLOW;
    /*  Konfigurieren der ADC Module
        SOC0 und SOC1 sind high priority, der Rest round robin  */

    AdcRegs.ADCCTL2.bit.CLKDIV4EN = 0;      //Sysclock
    AdcRegs.ADCCTL2.bit.CLKDIV2EN = 0;
    AdcRegs.INTSEL1N2.bit.INT1E = 1;        //Interrupt enabled
    AdcRegs.INTSEL1N2.bit.INT1SEL = 0;      //Primärstrom als Interruptquelle
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;     //Interruptflag muss rückgesetzt werden
    AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 2;  //Strom und Sekundärspannung high prio
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    //INT bei EOC, nicht SOC

    /*  Potentiometer   */
    AdcRegs.ADCSAMPLEMODE.bit.SIMULEN4 = 0;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 2;     //CPU-timer 1 als SOC0 Trigger
    AdcRegs.ADCSOC4CTL.bit.CHSEL = 0x5;
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 40;

    /*   Temperatur */
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 2;     //CPU-timer 1 als SOC0 Trigger
    AdcRegs.ADCSOC3CTL.bit.CHSEL = 0xB;
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 40;

    /*  Primärstrom   */
    AdcRegs.ADCSAMPLEMODE.bit.SIMULEN0 = 0;     //soc0-1 gleichzeitig (nope geht nicht nur A5 und B5 etc.)
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 0xD;
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 10;

    /*  Sekundärspannung    */
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 5;     //PWM1A als SOC0 Trigger
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x4;
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 10;

    /*  Primärspannung  */
    AdcRegs.ADCSAMPLEMODE.bit.SIMULEN2 = 0;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;     //PWM1A als SOC0 Trigger
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 0x3;
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 10;

    EDIS;
}

void ConfigSCIB(void)
{
    /*  Konfigurieren des SCIB Moduls. Zuständig
     *  für den Bluetooth Chip.
     *  (Vielleicht noch Transmitter Interrupt deak.)
     *  Optimierbar */

    ScibRegs.SCICTL1.bit.SWRESET = 0;       //Reset
    ScibRegs.SCICCR.bit.STOPBITS = 0;       //1 Stoppbit
    ScibRegs.SCICCR.bit.PARITYENA = 0;      //Parität DEaktiviert
    ScibRegs.SCICCR.bit.PARITY = 0;         //ungerade Parität
    ScibRegs.SCICCR.bit.SCICHAR = 7;        //8 bit pro Zeichen
    ScibRegs.SCICCR.bit.LOOPBKENA = 0;      //Loop back disabled
    ScibRegs.SCICCR.bit.ADDRIDLE_MODE = 0;  //Idle-Line Mode

    ScibRegs.SCICTL1.bit.RXENA = 1;         //receiver enabled
    ScibRegs.SCICTL1.bit.TXENA = 1;         //transmitter enabled
    ScibRegs.SCICTL1.bit.RXERRINTENA = 0;   //Receive error int disable
    ScibRegs.SCICTL1.bit.SLEEP = 0;         //sleep disabled
    ScibRegs.SCICTL1.bit.TXWAKE = 0;        //no transmitter wakeup mode
    ScibRegs.SCICTL2.bit.TXINTENA=0;        //disable transmitter interrupt

    ScibRegs.SCIHBAUD = 0;                  //Baudrate auf 115200 bit/s bei LSPCLK = 45MHz
    ScibRegs.SCILBAUD = 0x18;               //Baudrate auf 115200 bit/s bei LSPCLK = 45MHz

    /*  FIFO CONFIG */
    /*  Transmit    */
    ScibRegs.SCIFFTX.bit.SCIFFENA = 1;      //enable FIFO
    ScibRegs.SCIFFTX.bit.SCIRST = 1;        //unklar
    ScibRegs.SCIFFTX.bit.TXFIFOXRESET = 1;  //FIFO Zeiger-Operation
    ScibRegs.SCIFFTX.bit.TXFFIENA = 0;      //(vorerst) kein Interrupt
    ScibRegs.SCIFFTX.bit.TXFFIL = 0;        //wenn FIFO leer -> Interrupt

    /*  Receive */
    ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;   //RXFIFO Zeiger-Operation
    ScibRegs.SCIFFRX.bit.RXFFIENA = 1;      //Interrupt
    ScibRegs.SCIFFRX.bit.RXFFIL = 1;        //wenn RXFIFO mit 1 Zeichen belegt -> Interrupt

    ScibRegs.SCICTL1.bit.SWRESET=1; //Reset
}

void PushAsm(void)
{
    asm(" PUSH IER");   //IER-Einstellung in den Stack schreiben
    asm(" POP DBGIER"); //DBGIER-Einstellung aus dem Stack
    asm(" CLRC INTM");  //Freigabe aller Interrupts
    asm(" CLRC DBGM");  //Freigabe von Interrupts im Debugmode
}

void InitAll(void)
{
    SysCtrl();
    ConfigInterrupt();
    ConfigGpio();
    ConfigPwm();
    ConfigAdc();
    ConfigSCIB();
    PushAsm();     //call last
    //EnableInterrupts();

}


