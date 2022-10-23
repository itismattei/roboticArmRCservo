/*
 * servomotoreRC.h
 *
 *  Created on: 04 dic 2018
 *      Author: massimo
 */

#ifndef SERVOMOTORERC_H_
#define SERVOMOTORERC_H_

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif
//class servomotoreRC {
//public:
//	servomotoreRC(TIM_HandleTypeDef *);
//	~servomotoreRC();
//
//	int setPWM(uint32_t);
//	int setPWM2(uint32_t);
//
//	TIM_HandleTypeDef *datiPWM;
//	bool valid;
//	static int numMot;
//};
typedef enum{
	_base,
	_spalla,
	_gomito,
	_mano,
	_polso,
	_pinza
} tipoMotore;

typedef struct _servoRc{
	uint16_t 			numCHRc;		/// contiene il numero del canale del motore
	uint16_t 			delta;			/// contiene il valore attuale del PWM
	TIM_HandleTypeDef 	*TIM_PWM;		/// contiene l'indirizzo della struttura che ha i parametri del PWM
	uint32_t			periodo;
	tipoMotore			motore;
} servoRC;


/**
  * @brief  Imposta i servo RC.
  * @param  puntatore alle struttura del timer che fornisce le uscite PWM, numero del canale
  * @param  da impostare, elemento del vettore dei dati del servo da impostare.
  * @retval void
  */
void setRC(servoRC * RCptr, TIM_HandleTypeDef *datiPWM, int numCH, tipoMotore *tipo);
void initRC(servoRC * RCptr);
void goRC(servoRC * RCptr);

#ifdef __cplusplus
}
#endif

#endif /* SERVOMOTORERC_H_ */
