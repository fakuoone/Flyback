/*
 *  main.c
 *
 *  Created on: 20.03.2023
 *  Author: Fabian Kurthus
 *  Description: Flyback
 *  Features:
 */

#include <string.h>
#include <stdio.h>

#include "DSP28x_Project.h"
#include "init.h"
#include "functions.h"
#include "PI_controller.h"
#include "PL_controller.h"

#define PI_VOLT_KP 0.015
#define PI_VOLT_KI 10
#define PI_VOLT_TS 0.00002/3 // 20us abhängig von ADC-Int
#define PI_VOLT_LIMMAX 0.5
#define PI_VOLT_LIMMIN 0.0

#define PI_CURR_KP 0.01
#define PI_CURR_KI 7
#define PI_CURR_TS 0.00002/3 // 20us abhängig von ADC-Int
#define PI_CURR_LIMMAX 0.5
#define PI_CURR_LIMMIN 0.0

#define PL_VOLT_KBETA 1.2549
#define PL_VOLT_ALPHA1 -1.3589e-4
#define PL_VOLT_ALPHA2 3.059e-7
#define PL_VOLT_CW1 1.960527781858888e-04
#define PL_VOLT_CW2 1.921707243199674e-04
#define PL_VOLT_C1 -1.941376310081743
#define PL_VOLT_C2 0.941764533584249
#define PL_VOLT_LIMMAX 0.5
#define PL_VOLT_LIMMIN 0.0
#define PL_VOLT_TS 0.00002 // 20us abhängig von ADC-Int


/*  SH-Parameter Temperaturmessung  */
#define sh_A 0.001304
#define sh_B 0.0002352
#define sh_C 0.000000101

extern unsigned long int PWM_TBPRD;
extern unsigned long int PWM_TBPRD_2HZ;
extern unsigned long int PWM_CMP;
extern float f_TBCLK;

float d_soll = 0.1;
float f_soll = 150000.0;
float duty = 0;

char sci_data_out[21];
unsigned int sci_allow_transmit = 1;    // Freigabe durch ISR (wenn noch nicht alle Zeichen gesendet wurden)
unsigned int sci_enable_transmit = 0;   // Freigabe durch Programm
unsigned int sci_telegram_cnt = 0;
unsigned int read_sci_int_cnt = 0;
unsigned int timer1intcount = 0;
char sci_data_in[21];
unsigned int BLE_status = 0;    // 1 = connected, 0 = disconnected
unsigned int BLE_enable = 0;    // Freigabe über Bluetooth
unsigned int last_enable_btn_state = 0;

char char_sd_manual[] = "SDman";    // Shutdown per Hand
char char_sd_ble[] = "SDble";       // Shutdown durch BLE
char char_sd_curr[] = "SDcurr";     // Shutdown durch Überstrom
char char_sd_uvo[] = "SDuvo";       // Shutdown durch Unterspannung
char char_sd_ovo[] = "SDovo";       // Shutdown durch Überspannung sekundär
char char_sd_ovosec[] = "SDovo2";   // Shutdown durch Überspannung sekundär
char char_sd_otemp[] = "SDot";     // Shutdown durch Übertemperatur
char char_en_manual[] = "ENhand";   // Freigabe durch Hand
char char_en_ble[] = "ENble";       // Freigabe durch BLE
char char_ble_disconnect[] = "DCBLE";
char char_ble_connect[] = "COBLE";


/*  Systemzustände  */
float temp = 0;
float v_in = 0;
float v_out = 0;
float v_aim = 48;
float i_aim = 6;
float v_aim_ramp = 0;
float i_in = 0;
float i_in_integral = 0;

unsigned int enable_return = 0;
float last_duty = 0;
unsigned int reg_en = 1;
unsigned int volt_control = 0;
unsigned int current_control = 1;
unsigned int cnt_current_ISR = 0;

PI_controller PI_volt = {PI_VOLT_KP, PI_VOLT_KI, PI_VOLT_LIMMAX, PI_VOLT_LIMMIN, PI_VOLT_TS};
PI_controller PI_curr = {PI_CURR_KP, PI_CURR_KI, PI_CURR_LIMMAX, PI_CURR_LIMMIN, PI_CURR_TS};
PL_controller PL_volt = {PL_VOLT_KBETA, PL_VOLT_ALPHA1, PL_VOLT_ALPHA2, PL_VOLT_CW1, PL_VOLT_CW2, PL_VOLT_C1, PL_VOLT_C2, PL_VOLT_LIMMAX, PL_VOLT_LIMMIN, PL_VOLT_TS};

void main(void)
{
     InitAll();          //initalisiert allest
     PI_Init(&PI_volt);
     PI_Init(&PI_curr);
     PL_Init(&PL_volt);

     while (1) {asm(" IDLE");}
}


__interrupt void Timer1_ISR_Sys_control(void)
{
    float pot = (float)AdcResult.ADCRESULT4/4096;
    d_soll = pot / 2;
    temp = read_temperature(v_in, (float)AdcResult.ADCRESULT3/4096 * 3.3, sh_A, sh_B, sh_C);

    if (temp > 50) {enable_return = shutdown_Flyback(sci_data_out, char_sd_otemp, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);}

    /*  Sys Control */
    if (BLE_status == 1 && sci_enable_transmit) {create_transmit_data_block(sci_data_out, v_in, v_out, i_in, temp, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);}

    if (!enable_return && (GpioDataRegs.GPADAT.bit.GPIO8 || GpioDataRegs.GPADAT.bit.GPIO9) && ((GpioDataRegs.GPADAT.bit.GPIO7 && GpioDataRegs.GPADAT.bit.GPIO10 || BLE_enable)&& !last_enable_btn_state ))
    {
        sci_enable_transmit = 1;
        if (BLE_enable) {enable_return = enable_Flyback(sci_data_out, char_en_ble, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);}
        else {enable_return = enable_Flyback(sci_data_out, char_en_manual, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);}
    }

    BLE_enable = 0;
    update_status_leds((int)(duty * 10));

    if (!reg_en || !enable_return)
    {
        PI_Init(&PI_volt);
        PI_Init(&PI_curr);
        PL_Init(&PL_volt);
    }

    EPwm1Regs.CMPB = (int)(PWM_TBPRD * (0.5));    // Temperatur
    EPwm2Regs.CMPA.half.CMPA = (int)(PWM_TBPRD_2HZ * (1 - v_out / 55));    // Sekundärspannung
    EPwm3Regs.CMPA.half.CMPA = (int)(PWM_TBPRD_2HZ * (1 - v_in / 11));    // Primärspannung
    EPwm3Regs.CMPB = (int)(PWM_TBPRD_2HZ * (1 - duty));  // Duty

    last_enable_btn_state = (GpioDataRegs.GPADAT.bit.GPIO7 && GpioDataRegs.GPADAT.bit.GPIO10) || BLE_enable;    // Hilfsbit zur Flankenerkennung

    timer1intcount++;

}

__interrupt void CURRENT_ISR(void)
{
    GpioDataRegs.GPASET.bit.GPIO16 = 1;

    cnt_current_ISR++;

    i_in = 0.00161133 * (float)AdcResult.ADCRESULT0;
    v_out = 0.0148302 * (float)AdcResult.ADCRESULT1;
    v_in = 0.00298096 * (float)AdcResult.ADCRESULT2;

    i_in_integral += i_in;
    if (i_in_integral > 0) {i_in_integral -= 6;}

    if (i_in_integral > 1000)
    {
        enable_return = shutdown_Flyback(sci_data_out, char_sd_curr, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);
        GpioDataRegs.GPASET.bit.GPIO13 = 1; // Anzeige Überstrom
    }

    if (v_in > 11) {enable_return = shutdown_Flyback(sci_data_out, char_sd_ovo, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);}
    //if (v_in < 9) {enable_return = shutdown_Flyback(sci_data_out, char_sd_uvo, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);}
    //if (v_out > 57) {enable_return = shutdown_Flyback(sci_data_out, char_sd_ovosec, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);}

    if (enable_return)
    {
        if (reg_en)
        {
            /* Entscheidung ob Strom -oder Spannungsregelung    */
            if ((v_out < (v_aim - 10)) || (i_in_integral > 500) && !current_control)
            {
                current_control = 1;
                volt_control = 0;
            }

            if ((v_out > v_aim) && current_control)
            {
                volt_control = 1;
                current_control = 0;
            }

            if (current_control)
            {
                duty = PI_Update(&PI_curr, i_aim, i_in);
                GpioDataRegs.GPBSET.bit.GPIO53 = 1;
            }
            else if (volt_control)
            {
                //if (v_aim_ramp < v_aim) {v_aim_ramp += 0.001;}
                //else if (v_aim_ramp > v_aim) {v_aim_ramp -= 0.001;}
                duty = PI_Update(&PI_volt, v_aim, v_out);
                //duty = (float)PL_Update(&PL_volt, (double)v_aim, (double)v_out);
                GpioDataRegs.GPBCLEAR.bit.GPIO53 = 1;
            }
        }
        else {duty = d_soll;}

        EPwm1Regs.CMPA.half.CMPA = (int)(PWM_TBPRD * (1 - duty));
    }

    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;


    GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
}

__interrupt void SHUTDOWN_ISR(void)
{
    enable_return = shutdown_Flyback(sci_data_out, char_sd_manual, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);
    v_aim_ramp = 0;
    duty = 0;
    i_in_integral = 0;


    GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}

__interrupt void SCIB_receive_ISR(void)
{
    /*  SCIB-receive ISR ausgelöst durch Bluetooth    */
    char *sci_data_in_ptr = &sci_data_in[0];
    char temp[4][5];
    unsigned long i;
    unsigned int aim_temp = 0;
    unsigned int Kp_temp = 0;
    unsigned int Ki_temp = 0;
    unsigned int timer_temp = 40000;   // us

    read_sci_int_cnt = read_sci_data(sci_data_in_ptr, read_sci_int_cnt);

    //  Test: wenn vollständig empfangen
    if (read_sci_int_cnt == 0)
    {
        if (!strcmp(sci_data_in, "Connect!\x0d\x0a"))
        {
            BLE_status = 1;
            for (i = 0; i < 4000000; i++){} // Warteschleife, damit BLE die Daten erhält
            sci_enable_transmit = 1;
            ctrl_write_sci_data(sci_data_out, char_ble_connect, &sci_allow_transmit, &sci_enable_transmit,  &sci_telegram_cnt);
            sci_enable_transmit = 0;
        }
        else if (!strcmp(sci_data_in, "DisConnect!\x0d\x0a"))
        {
            for (i = 0; i < 4000000; i++){}
            ctrl_write_sci_data(sci_data_out, char_ble_disconnect, &sci_allow_transmit, &sci_enable_transmit,  &sci_telegram_cnt);
            BLE_status = 0;
            sci_enable_transmit = 0;
        }
        else if (!strcmp(sci_data_in, "Enable")) {BLE_enable = 1;}
        else if (!strcmp(sci_data_in, "Disable"))
        {
            enable_return = shutdown_Flyback(sci_data_out, char_sd_ble, &sci_allow_transmit, &sci_enable_transmit, &sci_telegram_cnt);}

        if (((sci_data_in[0] - '0') > 0) && ((sci_data_in[0] - '0') <= 9))
        {
            for (i = 0; i < 4; i++)
            {
                strcpy(temp[i], &sci_data_in[1 + i*4]);
                temp[i][4] = '\x00';
            }

            sscanf(temp[1], "%d", &Kp_temp);
            sscanf(temp[2], "%d", &Ki_temp);
            sscanf(temp[3], "%d", &timer_temp);
            sscanf(temp[0], "%d", &aim_temp);

            switch (sci_data_in[0])
            {
            case '1':   // Spannungsregler über Bluetooth
                reg_en = 1;
                v_aim = (float)aim_temp / 100;
                if (v_aim > 55) {v_aim = 55;}
                if (v_aim < 15) {v_aim = 15;}
                PI_volt.Kp = (float)Kp_temp / 1000;
                PI_volt.Ki = (float)Ki_temp;
                //ConfigCpuTimer(&CpuTimer1, 90, (float)(1000 * timer_temp));
                //CpuTimer1Regs.TCR.bit.TSS = 0;
                break;

            case '2':   // Stromregler über Bluetooth
                reg_en = 1;
                i_aim = (float)aim_temp / 1000;
                if (i_aim > 4) {i_aim = 4;}
                if (i_aim < 0) {i_aim = 0;}
                PI_curr.Kp = (float)Kp_temp / 1000;
                PI_curr.Ki = (float)Ki_temp;
                //ConfigCpuTimer(&CpuTimer1, 90, (float)(1000 * timer_temp));
                //CpuTimer1Regs.TCR.bit.TSS = 0;
                break;

            case '3':   // Regler deaktiviert, Vorgabe durch duty cycle
                reg_en = 0;
                d_soll = (float)aim_temp / 1000;
                if (d_soll > 0.55) {d_soll = 0.55;}
                if (d_soll < 0) {d_soll = 0;}
                //ConfigCpuTimer(&CpuTimer1, 90, (float)(1000 * timer_temp));
                //CpuTimer1Regs.TCR.bit.TSS = 0;
                break;

            default:
                break;
            }
        }
    }

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



