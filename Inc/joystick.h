/*
 * joystick.h
 *
 *  Created on: 19 mar 2019
 *      Author: s_rvrndr02a27g479k
 *
 *
 *	Il joystick e' realizzato tramite 2 potenziometri ed un interruttore che e' in grado di rilevare la
 *	pressione dello stesso.
 *	I potenziometri, per compatibilita' arduino possono essere messi sui pin A0...A5 della scheda.
 *	Si puo' pensare di utilizzare A0, A1 per il joystick di sinistra ed A2, A3 per quello di destra.
 *	Siccome la lettura dei canali AD avviene in sequwenza e con trasferimento a DMA, l'avvio della
 *	lettura avviene con la chiamata: HAL_ADC_Start_DMA(&hadc3, buffer, 6); che trasferisce in buffer
 *	i valori dei 6 canali, rispettivamente buffer[0] = A0, ... buffer[5] = A5
 *
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_



#include "math.h"
#include <stdint.h>
#include <stdbool.h>

extern uint32_t buffer[];  // define variables

///
/// strcu che contiene i dati della posizione dei potenziometri
typedef struct __joystick {

	uint16_t pinVRx;
    uint16_t pinVRy;
    uint16_t pinSW;
    int  zeroX; //valore asse x quando il potenziomentro è in posizione (0,0)
    int  zeroY; //valore asse y quando il potenziomentro è in posizione (0,0)

    uint16_t numPot;

} joystick;

typedef enum{
	CV,
	CCV,
	NO } verse;

	void joystickINIT (joystick *, uint8_t VRx, uint8_t VRy, uint8_t SW,
			int valX, int valY, uint16_t ch);

    int  getDegree  ( joystick *);
    bool push       ( joystick *);
    verse getVerseX ( joystick *);
    verse getVerseY ( joystick *);


#endif /* JOYSTICK_H_ */
