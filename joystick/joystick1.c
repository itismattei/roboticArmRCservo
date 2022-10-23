/*
 * joystick1->c
 *
 *  Created on: 19 mar 2019
 *      Author: s_rvrndr02a27g479k
 *
 *	/// il funzionamento potrebbe essere questo:
 *	il joystick di sinistra, potenziometro orizzontale ruota la spalla; il potenziometro verticale ruota il
 *	braccio
 *	il joystick di destra , potenziometro orizzontale ruota l'avambraccio; il potenziometro verticale inclina
 *	il polso
 *	premendo il joystick di destra, il potenziometro orizzonatale ruota il polso e quello verticale apre la
 *	pinza.
 */


#include "math.h"
#include <stdbool.h>
#include "main.h"
#include "stm32f7xx_hal.h"

#include "joystick.h"


void joystickINIT ( joystick *newJoy, uint8_t valueX, uint8_t valueY,
		uint8_t valueSw, int zeroX, int zeroY, uint16_t ch ){
  newJoy->pinVRx = valueX;
  newJoy->pinVRy = valueY;
  newJoy->pinSW= valueSw;
  newJoy->zeroX= zeroX;
  newJoy->zeroY= zeroY;
//  pinMode( newJoy->pinVRx, INPUT);
//  pinMode( newJoy->pinVRy, INPUT);
//  pinMode( newJoy->pinSw,INPUT_PULLUP);
  /// associa la struttura al numero del potenziometro
  newJoy->numPot = ch;
};

bool push ( joystick *newJoy ){
  bool state;
  if(HAL_GPIO_ReadPin(GPIOC, newJoy->pinSW )== GPIO_PIN_SET)
    state = false;
  else
    state = true;
  return state;
};

///
/// legge il valore del convertitore AD su cui e' posto il potenziometro e ne trova l'angolo rispetto
/// alla posizione di riposo. Nella posizione di riposo la lettura dell'A/D dovrebbe essere
/// 2047

extern ADC_HandleTypeDef hadc3;
extern bool ADupdate;

int getDegree( joystick *newJoy ){
	int valVRX;
	if (ADupdate == true){
		ADupdate = false;
		switch(newJoy->numPot){
		case 0:

		break;

		case 1:
			valVRX = buffer[1] - 2047; ///HAL_GPIO_ReadPin(newJoy->pinVRx)-newJoy->zeroX;
		break;

		case 2:

		break;

		case 3:

		break;


		case 4:

		break;

		case 5:

		break;
		}

		int valVRY  = 0; ///HAL_GPIO_ReadPin(newJoy->pinVRy)-newJoy->zeroY;
		int radiant;
		if (valVRY != 0)
		  radiant = atan(valVRX/valVRY);


		int degree  = (int) (radiant * 180.0)/ M_PI;

		if(newJoy->pinVRx<0 && newJoy->pinVRy>0) degree=degree+180;//si trova nel secondo quadrante
		if(newJoy->pinVRx<0 && newJoy->pinVRy<0) degree=degree+180;//si trova nel terzo quadrante
		if(newJoy->pinVRx>0 && newJoy->pinVRy<0) degree=degree+360;//si trova nel quarto quadrante
		return degree;
	}
	else
		return -10000;
};

verse getVerseX ( joystick * newJoy ){
  verse way;
  if ( newJoy->pinVRx == 0 ) way = NO;
  if ( newJoy->pinVRx >  0 ) way = CV;
  if ( newJoy->pinVRx <  0 ) way = CCV;
  return way;
};

verse getVerseY ( joystick* newJoy ){
  verse way;
  if ( newJoy->pinVRy == 0 ) way = NO;
  if ( newJoy->pinVRy >  0 ) way = CV;
  if ( newJoy->pinVRy <  0 ) way = CCV;
  return way;
};


