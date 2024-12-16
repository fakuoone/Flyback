
/*
 * functions.c
 *
 *  Created on: 24.06.2022
 *      Author: Fabian
 */

#include "DSP28x_Project.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/*  Globale Variablen   */
extern unsigned long int PWM_TBPRD;
extern unsigned int PWM_CMP;


float read_temperature(float priVoltage, float ADCVoltage, float A, float B, float C)
{
    /*  Berechnung der Temperatur nach S-H-Modell   */
    float NTC_Voltage = ADCVoltage;
    float Resistance = 16000 * NTC_Voltage / (priVoltage - NTC_Voltage);  // Spannungsteiler mit 1,6k zu Vprim
    return 1 / (A + B * log(Resistance) + C * log(Resistance) * log(Resistance) * log(Resistance)) - 273.15;
}

unsigned int read_sci_data(char *data_storage, unsigned int int_cnt)
{
    /*  Liest vollen SCIRX FIFO in den char-Array auf den der Pointer zeigt */
    unsigned int return_val = 0;
    unsigned int temp_buf = 0;
    unsigned int i;

    temp_buf = ScibRegs.SCIRXBUF.bit.RXDT;
    if (int_cnt < 20)
    {
        *(data_storage + int_cnt) = temp_buf;
        return_val = int_cnt + 1;
    }
    else
    {
        return_val = 0;
    }


    if ((temp_buf == 10) || (temp_buf == 0x00))
    {
        return_val = 0;
        for (i = int_cnt + 1; i < 16; i++) {*(data_storage + i) = 0x00;}
    }

    return return_val;
}

unsigned int write_sci_data(char *data_storage, unsigned int allow_transmit)
{
    /*  Sendet Zeichen aus *data_storage  */
    unsigned int i;
    unsigned int status = 0;

    for (i = 0; i < 4; i++)
    {
        if (*data_storage != 0x00)
        {
            ScibRegs.SCITXBUF = *data_storage;
            allow_transmit = 0;
            data_storage++;
        }
        else
        {
            status = 1; //1 = letztes Zeichen erreicht
            allow_transmit = 2;
            i = 4;
            ScibRegs.SCIFFTX.bit.TXFFIENA = 0;  //deaktiviert TXFIFOINT
        }
    }

    return allow_transmit + status;
}

void ctrl_write_sci_data(char * char_out, char * char_send, unsigned int * allow_transmit, unsigned int * enable_transmit, unsigned int * telegram_cnt)
{
    /*  Übergelagerte Funktion von write_sci_data. Beschreibt char_out und passt entsprechnde Kontrollvariablen an  */
    if (*allow_transmit && *enable_transmit)
    {
        strcpy(char_out, char_send);
        *telegram_cnt = 0;
        ScibRegs.SCIFFTX.bit.TXFFIENA = 1;
    }
}

unsigned int copy_string(char *data_storage, char *new_data_storage)
{
    /*  Ändert den String in *data_storage zu *new_data_storage
     *  Annahme sind gleichlange strings    */
    unsigned int i;
    unsigned int data_storage_length = strlen(new_data_storage);

    for (i = 0; i < (data_storage_length + 1); i++)
    {
        *(data_storage + i) = *(new_data_storage + i);
    }

    return 0;
}

void update_status_leds(unsigned int bit_pattern)
{
    /*  Ändert die Status-LEDs  */
    unsigned int control[6] = {0, 0, 0, 0, 0, 0};
    if (bit_pattern > 0){control[bit_pattern - 1] = 1;}

    GpioDataRegs.GPASET.bit.GPIO17 = control[1];
    GpioDataRegs.GPBSET.bit.GPIO44 = control[0];
    GpioDataRegs.GPBSET.bit.GPIO50 = control[2];
    GpioDataRegs.GPBSET.bit.GPIO51 = control[3];
    GpioDataRegs.GPBSET.bit.GPIO52 = control[4];
    GpioDataRegs.GPBSET.bit.GPIO53 = control[5];

    GpioDataRegs.GPACLEAR.bit.GPIO17 = !control[1];
    GpioDataRegs.GPBCLEAR.bit.GPIO44 = !control[0];
    GpioDataRegs.GPBCLEAR.bit.GPIO50 = !control[2];
    GpioDataRegs.GPBCLEAR.bit.GPIO51 = !control[3];
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = !control[4];
    GpioDataRegs.GPBCLEAR.bit.GPIO53 = !control[5];
}

void create_transmit_data_block(char *char_out, float voltage_in, float voltage_out, float current_in, float temperature, unsigned int * allow_transmit, unsigned int * enable_transmit, unsigned int * telegram_cnt)
{
    unsigned int str_len = 0;
    unsigned int i;

    char temp_str[5];
    char v_out[5];
    char v_in[5];
    char i_in[5];
    char Temp[5];

    /*  Erzeugt den zyklischen Datenstream an den Bluetooth Chip    */
    snprintf(v_in, 5, "%d", (int)(100 * voltage_in));
    str_len = strlen(v_in) + 1;
    copy_string(&temp_str[5-str_len], &v_in[0]);
    for (i = 0; i < (5 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[0], &temp_str[0]);

    snprintf(v_out, 5, "%d", (int)(100 * voltage_out));
    str_len = strlen(v_out) + 1;
    copy_string(&temp_str[5-str_len], &v_out[0]);
    for (i = 0; i < (5 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[4], &temp_str[0]);

    snprintf(i_in, 5, "%d", (int)(1000 * current_in));
    str_len = strlen(i_in) + 1;
    copy_string(&temp_str[5-str_len], &i_in[0]);
    for (i = 0; i < (5 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[8], &temp_str[0]);

    snprintf(Temp, 5, "%d", (int)(100 * temperature));
    str_len = strlen(Temp) + 1;
    copy_string(&temp_str[5-str_len], &Temp[0]);
    for (i = 0; i < (5 - str_len); i++) {temp_str[i] = '0';}
    copy_string(&char_out[12], &temp_str[0]);

    ctrl_write_sci_data(char_out, char_out, allow_transmit, enable_transmit, telegram_cnt);
}

unsigned int shutdown_Flyback(char * char_out, char * char_sd_type, unsigned int * allow_transmit, unsigned int * enable_transmit, unsigned int * telegram_cnt)
{
    EPwm1Regs.ETSEL.bit.SOCAEN = 0; //ADC-Interrupt deaktiviert
    EPwm1Regs.AQCTLA.bit.CAU = 1;   //EPWM1A low wenn Zähler CMPA erreicht (deaktiviert PWM)
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
    update_status_leds(0);

    ctrl_write_sci_data(char_out, char_sd_type, allow_transmit, enable_transmit, telegram_cnt);
    *enable_transmit = 0;   // Deaktivieren der Sendeberechtigung wenn Flyback deaktiviert

    return 0;
}

unsigned int enable_Flyback(char * char_out, char * char_en_type, unsigned int * allow_transmit, unsigned int * enable_transmit, unsigned int * telegram_cnt)
{
    EPwm1Regs.ETSEL.bit.SOCAEN = 1; //ADC-Interrupt aktiviert
    EPwm1Regs.AQCTLA.bit.CAU = 2;   //EPWM1A high wenn Zähler CMPA erreicht
    GpioDataRegs.GPASET.bit.GPIO3 = 1;

    ctrl_write_sci_data(char_out, char_en_type, allow_transmit, enable_transmit, telegram_cnt);

    return 1;
}

