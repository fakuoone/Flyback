/*
 * pl_controller.c
 *
 *  Created on: 09.04.2023
 *  Author: Fabianus
 */

#include "PL_controller.h"
#include "DSP28x_Project.h"


void PL_Init(PL_controller *pl)
{
    /*  Initialisiert den Regler    */
    (*pl).yk_1 = 0.0;
    (*pl).yk = 0.0;
    (*pl).pk_1 = 0.0;
    (*pl).pk = 0.0;
    (*pl).ok_1 = 0.0;
    (*pl).ok = 0.0;
    (*pl).qk = 0.0;
}


float PL_Update(PL_controller *pl, double soll, double ist)
{
    /*  Berechnet den aktuellen Regler-Output   */
    pl->ok = -pl->c2 * ist - pl->alpha2 * pl->qk;
    pl->pk = pl->cw2 * soll + pl->ok_1 - pl->c1 * ist - pl->alpha1 * pl->qk;
    pl->qk = pl->cw1 * soll + pl->pk_1 - ist;
    pl->yk = pl->yk_1 + pl->kbeta * pl->qk ;

    /*  Beschränkung der Regler-Ausgangsgröße   */
    if (pl->yk > pl->limMax){pl->yk = pl->limMax;}
    if (pl->yk < pl->limMin){pl->yk = pl->limMin;}

    /*  Aktualisierung der Differenzenwerte */
    pl->yk_1 = pl->yk;
    pl->pk_1 = pl->pk;
    pl->ok_1 = pl->ok;

    return pl->yk;
}

