/*
 *  main.c
 *
 *  Created on: 11.04.2022
 *  Author: Fabian Kurth
 *  Description: main control file for a 3 phase generator system
 *  Features:
 */

#include <string.h>
#include <stdio.h>

#include "DSP28x_Project.h"
#include "init.h"
#include "functions.h"
#include "PI_controller.h"

#define PI_VOLT_KP 0.000025
#define PI_VOLT_KI 0.05
#define PI_VOLT_Ts 0.00002 // 20us abhängig von ADC-Int
#define PI_VOLT_LIMMAX 0.5
#define PI_VOLT_LIMMIN 0.01


/*  SH-Parameter Temperaturmessung  */
#define sh_A 0.001304
#define sh_B 0.0002352
#define sh_C 0.000000101

extern unsigned long int PWM_TBPRD;
extern unsigned long int PWM_TBPRD_2HZ;
extern unsigned long int PWM_CMP;
extern float f_TBCLK;

float d_soll = 0.0;
float f_soll = 150000.0;
float duty = 0.0;

char sci_data_out[21];
unsigned int sci_allow_transmit = 1;
unsigned int sci_telegram_cnt = 0;
unsigned int read_sci_int_cnt = 0;
unsigned int timer1intcount = 0;
char sci_data_in[16];

/*  Systemzustände  */
float temp = 0;
float v_in = 0;
float v_out = 0;
float v_aim = 48.0;
float i_in = 0;
unsigned int v_in_raw = 0;
unsigned int v_out_raw = 0;
unsigned int i_in_raw = 0;

unsigned int enable_return = 0;
float last_duty = 0;
unsigned int reg_en = 1;

PI_controller PI_volt = {PI_VOLT_KP, PI_VOLT_KI, PI_VOLT_LIMMAX, PI_VOLT_LIMMIN, PI_VOLT_Ts};

void main(void)
{
     InitAll();          //initalisiert allest
     PI_Init(&PI_volt);


     while (1) {asm(" IDLE");}
}


__interrupt void Timer1_ISR_Sys_control(void)
{
    /*  Sys Control */
    if (sci_allow_transmit && (timer1intcount > 0))
    {
        timer1intcount = 0;
        create_transmit_data_block(sci_data_out, v_in, v_out, i_in, temp);
        sci_telegram_cnt = 0;
        ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
    }

    if ((GpioDataRegs.GPADAT.bit.GPIO8 || GpioDataRegs.GPADAT.bit.GPIO9) && GpioDataRegs.GPADAT.bit.GPIO7 && GpioDataRegs.GPADAT.bit.GPIO10)
    {
        enable_return = enable_Flyback();
    }

    if (!enable_return)
    {
        last_duty = 0;
        d_soll = 0;
        duty = 0;
    }

    update_status_leds((int)(duty * 10));
    EPwm1Regs.CMPB = (int)(PWM_TBPRD * (0.5));    // Temperatur
    EPwm2Regs.CMPA.half.CMPA = (int)(PWM_TBPRD_2HZ * (1 - v_out / 3.3));    // Sekundärspannung
    EPwm3Regs.CMPA.half.CMPA = (int)(PWM_TBPRD_2HZ * (1 - v_in / 3.3));    // Primärspannung
    EPwm3Regs.CMPB = (int)(PWM_TBPRD_2HZ * (1 - duty));  // Duty

    timer1intcount++;
}

__interrupt void CURRENT_ISR(void)
{
    i_in = 0.001611 * (float)AdcResult.ADCRESULT0;
    v_out = 0.01483 * (float)AdcResult.ADCRESULT1;
    v_in = 0.002981 * (float)AdcResult.ADCRESULT2;

    if (enable_return)
    {
        duty = PI_Update(&PI_volt, v_aim, v_out);
        EPwm1Regs.CMPA.half.CMPA = (int)(PWM_TBPRD * (1 - duty));
    }
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

__interrupt void SHUTDOWN_ISR(void)
{
    enable_return = shutdown_Flyback();
    PI_Init(&PI_volt);
    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

__interrupt void SCIB_receive_ISR(void)
{
    /*  SCIB-receive ISR ausgelöst durch Bluetooth    */
    char *sci_data_in_ptr = &sci_data_in[0];

    read_sci_int_cnt = read_sci_data(sci_data_in_ptr, read_sci_int_cnt);

    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;

}

__interrupt void SCIB_transmit_ISR(void)
{
    /*  SCIB-send ISR ausgelöst durch leeren TXFIFO    */
    char *sci_data_out_ptr = &sci_data_out[4 * sci_telegram_cnt];
    unsigned int write_status_return = 0;
    unsigned int write_status = 0;
    unsigned int write_allow_transmit_return = 0;

    sci_telegram_cnt++;

    write_status_return = write_sci_data(sci_data_out_ptr, sci_allow_transmit);

    write_status = write_status_return & 1;
    write_allow_transmit_return = (write_status_return & 2) >> 1;

    sci_allow_transmit = write_allow_transmit_return;

    if (write_status){sci_telegram_cnt = 0;}

    ScibRegs.SCIFFTX.bit.TXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.bit.ACK9 = 1;

}



