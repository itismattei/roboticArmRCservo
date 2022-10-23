/*
 * RCsm.cpp
 *
 *  Created on: 24 nov 2019
 *      Author: massimo
 */

#include <RCsm.h>
#include <stdio.h>

RCsm::RCsm() {
	// TODO Auto-generated constructor stub
	numCHRc	=	0;		/// contiene il numero del canale del motore
	delta	=	0;			/// contiene il valore attuale del PWM
	TIM_PWM =	NULL;		/// contiene l'indirizzo della struttura che ha i parametri del PWM
	periodo	=	0;
	motore	= 	base;

}

RCsm::~RCsm() {
	// TODO Auto-generated destructor stub
}


/**
  * @brief  Questa funzione inizializza i parametri per ciascun servo.
  * @param  gestore del timer.
  * @param  numero del canale come riportato sulla scheda di comando.
  * @param  tipo di motore (o grado di libertà) secondo la struttura in RCsm.h
  * @retval stato (errore o ok)
  */
uint8_t RCsm::numPWM = 0;

int RCsm::initRC(TIM_HandleTypeDef *datiPWM, int numCH, tipoMotore1 tipo){
	if (datiPWM == NULL){
		TIM_PWM = NULL;
		return NO_TIM;
	}
	//! Il campo Period contiene il valore del periodo del PWM che in questa applicazione
	//! e' impostato a 2000 conteggi. E' possibile quindi avere un delta che va da 0 a 1999
	//! con una risoluzione di 1/2000 = 0,05%  che e' un valore sicuramente adatto per questo
	//! progetto.
	uint32_t perPWM = datiPWM->Init.Period;
	TIM_PWM = datiPWM;
	numCHRc =  numCH;
	/// imposta il PWM al 7.5% in modo da posizionare il braccio in posizione circa verticale
	//! il 7,5% del periodo e' Periodo * 0,075
	delta = (uint32_t) perPWM * 0.075;
	periodo = perPWM;
	//! spegne i PWM
	//! i PWM vengono impostati ma al momento vengono spenti,
	//! poiche' i registri a seguire sono quelli che impostano il delta
	switch(numCH){
	case 1:
		TIM_PWM->Instance->CCR1 = 0; //
		HAL_TIM_PWM_Start(datiPWM, TIM_CHANNEL_1);
	break;

	case 2:
		TIM_PWM->Instance->CCR2 = 0;
		HAL_TIM_PWM_Start(datiPWM, TIM_CHANNEL_2);
	break;

	case 3:
		TIM_PWM->Instance->CCR3 = 0;
		HAL_TIM_PWM_Start(datiPWM, TIM_CHANNEL_3);
	break;

	case 4:
		TIM_PWM->Instance->CCR4 = 0;
		HAL_TIM_PWM_Start(datiPWM, TIM_CHANNEL_4);
	break;
	}
	//! inoltre viene dato lo start al pwm in modo che alla prossima scrittura in CCRx
	//! si avra' l'avvio dei servi

	/// infine aggiorna il numero dei motori gia' impostati
	numPWM++;
	return RC_OK;
}

/**
  * @brief  Questa funzione scrive il valore del pwm nell'apposito registro del processore.
  * @param  nessuno.
  * @retval stato della funzione.
  */
int RCsm::go(void){
	/// delta deve essere appropriato
	if (TIM_PWM == NULL){
		//! Non inizializzato
		printf("Struttura dati PWM non inizializzata!\n");
		return NO_TIM;
	}
	//! controlla delta nell'intervallo 5% - 10%
	int inf, sup;
	switch(motore){
		case base:

			inf = (int) (periodo * 0.040);
			sup = (int) (periodo * 0.130);
		break;

		case gomito:

			inf = (int) (periodo * 0.040);
			sup = (int) (periodo * 0.127);
		break;

		case spalla:

			inf = (int) (periodo * 0.040);
			sup = (int) (periodo * 0.110);
		break;

		case mano:

			inf = (int) (periodo * 0.040);
			sup = (int) (periodo * 0.128);
		break;

		case polso:

			inf = (int) (periodo * 0.050);
			sup = (int) (periodo * 0.1145);
		break;

		case pinza:

			inf = (int) (periodo * 0.040);
			sup = (int) (periodo * 0.073);
		break;

	}

	if (delta < inf || delta > sup){
		//! esterno all'intervallo di controllo del servoRC
		printf("PWM esterno all'intervallo 5%% - 10%%\n");
		return RANGE_PWM;
	}
	else
		//! si imposta il delta del PWM ed il segnale esce dal pin
		switch(numCHRc){
		case 1:
			TIM_PWM->Instance->CCR1 = delta;
		break;

		case 2:
			TIM_PWM->Instance->CCR2 = delta;
		break;

		case 3:
			TIM_PWM->Instance->CCR3 = delta;
		break;

		case 4:
			TIM_PWM->Instance->CCR4 = delta;
		break;

		default:
			printf("Numero del pwm errato\n");
		}

	return RC_OK;

}

int RCsm::assignMotor(){

	return 0;

}
