/*
 * PL_controller.h
 *
 *  Created on: 09.04.2023
 *  Author: Fabian
 */

#ifndef PL_CONTROLLER_H_
#define PL_CONTROLLER_H_

typedef struct
{
    double kbeta, alpha1, alpha2, cw1, cw2, c1, c2;

    double limMax;   //highest output
    double limMin;   //lowest output

    double Ts;        //sample Time
    // Variablen siehe Grafik PL-Regler
    double ok;
    double ok_1;
    double pk;
    double pk_1;
    double qk;

    double yk_1;     //previous output
    double yk;       //output
} PL_controller;

void PL_Init(PL_controller *pi);
float PL_Update(PL_controller *pi, double soll, double ist);

#endif /* PL_CONTROLLER_H_ */
