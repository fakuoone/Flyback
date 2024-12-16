/*
 * functions.h
 *
 *  Created on: 24.06.2022
 *      Author: Fabianus
 */
#include <stdbool.h>


#ifndef FUNCTIONS_H
#define FUNCTIONS_H


/*  uC Prototypes  */
/*  Funktional  */
unsigned int read_sci_data(char *data_storage, unsigned int length);
unsigned int write_sci_data(char *data_storage, unsigned int allow_transmit);
void ctrl_write_sci_data(char * char_out, char * char_send, unsigned int * allow_transmit, unsigned int * enable_transmit, unsigned int * telegram_cnt);
unsigned int copy_string(char *data_storage, char *new_data_storage);
void create_transmit_data_block(char *char_out, float voltage_in, float voltage_out, float current_in, float temperature, unsigned int * allow_transmit, unsigned int * enable_transmit, unsigned int * telegram_cnt);
void update_status_leds(unsigned int bit_pattern);
float  read_temperature(float priVoltage, float ADCVoltage, float A, float B, float C);
unsigned int enable_Flyback(char *char_out, char *char_en_type, unsigned int *allow_transmit, unsigned int *enable_transmit, unsigned int *telegram_cnt);
unsigned int shutdown_Flyback(char *char_out, char *char_sd_type, unsigned int *allow_transmit, unsigned int *enable_transmit, unsigned int *telegram_cnt);


#endif /* FUNCTIONS_H_ */
