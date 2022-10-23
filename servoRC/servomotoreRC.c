/*
 * servomotoreRC.cpp
 *
 *  Created on: 04 dic 2018
 *      Author: massimo
 */

#include <servomotoreRC.h>
#include <stdlib.h>

servoRC RC[6] = {0};

//int servomotoreRC::numMot = 0;
//
//servomotoreRC::servomotoreRC(TIM_HandleTypeDef *p) {
//	// TODO Auto-generated constructor stub
//	datiPWM = p;
//	/// incrementa il valore della variabile statica in modo da conoscere i
//	/// canali PWM che ha gia' assegnato
//	numMot++;
//	if (datiPWM->Instance == TIM3){
//		/// vengono definiti due soli canali
//		if (numMot > 2)
//			valid = false;
//		else
//			valid = true;
//	}
//	else if (datiPWM->Instance == TIM4){
//		/// vengono definiti 4 canali PWM
//		if(numMot > 4)
//			valid = true;
//		else
//			valid = false;
//	}
//
//}
//
//servomotoreRC::~servomotoreRC() {
//	// TODO Auto-generated destructor stub
//}
//
/////
///// imposta il PWM del motore sul relativo canale
///// lo stesso metodo deve riconoscere a quale servomotore e quindi quale
///// canale va applicato
/////
//int servomotoreRC::setPWM(uint32_t val){
//	if (datiPWM != NULL)
//		switch(numMot){
//		case 1:
//			datiPWM->Instance->CCR1 = val;
//		break;
//
//		case 2:
//			datiPWM->Instance->CCR2 = val;
//		break;
//
//		case 3:
//			if (valid)
//				datiPWM->Instance->CCR3 = val;
//		break;
//
//		case 4:
//			if (valid)
//				datiPWM->Instance->CCR4 = val;
//		break;
//
//
//		default:
//		break;
//		}
//
//	return val;
//}
//
//int servomotoreRC::setPWM2(uint32_t val){
//	if (datiPWM != NULL)
//		datiPWM->Instance->CCR2 = val;
//	return val;
//}

/**
  * @brief  Imposta i servo RC al valore del 7.5% tuttaiva non avvia ancora i PWM.
  * @param  puntatore alle struttura del timer che fornisce le uscite PWM.
  * @retval void
  */
void setRC(servoRC * RCptr, TIM_HandleTypeDef *datiPWM, int numCH, tipoMotore *tipo){
	/// controllo di validita' del dato ricevuto
	if (datiPWM == NULL){
		RCptr->TIM_PWM = NULL;
		return;
	}

	//! Il campo Period contiene il valore del periodo del PWM che in questa applicazione
	//! e' impostato a 2000 conteggi. E' possibile quindi avere un delta che va da 0 a 1999
	//! con una risoluzione di 1/2000 = 0,05%  che e' un valore sicuramente adatto per questo
	//! progetto.
	uint32_t perPWM = datiPWM->Init.Period;
	RCptr->TIM_PWM = datiPWM;
	RCptr->numCHRc =  numCH;

	/// imposta il PWM al 7.5% in modo da posizionare il braccio in posizione circa verticale
	//! il 7,5% del periodo e' Periodo * 0,075
	RCptr->delta = (uint32_t) perPWM * 0.075;

	RCptr->periodo = perPWM;
	//! spegne i PWM
	//! i PWM vengono impostati ma al momento vengono spenti,
	//! poich� i registri a seguire sono quelli che impostano il delta
	switch(numCH){
	case 1:
		RCptr->TIM_PWM->Instance->CCR1 = 0; //
		HAL_TIM_PWM_Start(datiPWM, TIM_CHANNEL_1);
	break;

	case 2:
		RCptr->TIM_PWM->Instance->CCR2 = 0;
		HAL_TIM_PWM_Start(datiPWM, TIM_CHANNEL_2);
	break;

	case 3:
		RCptr->TIM_PWM->Instance->CCR3 = 0;
		HAL_TIM_PWM_Start(datiPWM, TIM_CHANNEL_3);
	break;

	case 4:
		RCptr->TIM_PWM->Instance->CCR4 = 0;
		HAL_TIM_PWM_Start(datiPWM, TIM_CHANNEL_4);
	break;
	}
	//! inoltre viene dato lo start al pwm in modo che alla prossima scrittura in CCRx
	//! si avra' l'avvio dei servi

}

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/**
  * @brief  Imposta i servo RC.
  * @param  puntatore alle struttura del timer che fornisce le uscite PWM.
  * @retval void
  */
void initRC(servoRC * RCptr){
	  setRC(&RC[3], &htim1, 1, _mano);  /// corrisponde al quarto connettore (M4) della scheda arancione
	  setRC(&RC[4], &htim1, 2, _polso);  /// corrisponde al quinto connettore (M5) della scheda arancione
	  setRC(&RC[5], &htim1, 3, _pinza);  /// corrisponde al sesto connettore (M6) della scheda arancione
	  // inizializza il motore posto su  TIM3  CH2 ( PA7)
	  //setRC(&RC[3], &htim3, 1);
	  setRC(&RC[0], &htim3, 2, _base);  /// corrisponde al primo connettore (M1) della scheda arancione
	  // inizializza i motori posti su TIM4 CH3 e CH4 (PD14 e PD15)
	  setRC(&RC[1], &htim4, 3, _gomito);  /// corrisponde al secondo connettore (M2) della scheda arancione
	  setRC(&RC[2], &htim4, 4, _spalla);  /// corrisponde al terzo connettore (M3) della scheda arancione
}

/**
  * @brief  Imposta il delta del PWM e attiva l'uscita
  * @param  puntatore alle struttura del timer che fornisce le uscite PWM, numero del canale, delta.
  * @retval void
  */
void goRC(servoRC * RCptr){

	/// delta deve essere appropriato
	if (RCptr->TIM_PWM == NULL){
		//! Non inizializzato
		printf("Struttura dati PWM non inizializzata!\n");
		return;
	}
	//! controlla delta nell'intervallo 5% - 10%
	uint32_t periodo = RCptr->periodo;
	int inf, sup, delta;
	switch(RCptr->motore){
			case _base:
				delta = RCptr->delta;
				inf = (int) (periodo * 0.040);
				sup = (int) (periodo * 0.130);
			break;

			case _gomito:
				delta = RCptr->delta;
				inf = (int) (periodo * 0.040);
				sup = (int) (periodo * 0.127);
			break;

			case _spalla:
				delta = RCptr->delta;
				inf = (int) (periodo * 0.040);
				sup = (int) (periodo * 0.110);
			break;

			case _mano:
				delta = RCptr->delta;
				inf = (int) (periodo * 0.040);
				sup = (int) (periodo * 0.128);
			break;

			case _polso:
				delta = RCptr->delta;
				inf = (int) (periodo * 0.050);
				sup = (int) (periodo * 0.1145);
			break;

			case _pinza:
				delta = RCptr->delta;
				inf = (int) (periodo * 0.040);
				sup = (int) (periodo * 0.073);
			break;

	}

	if (delta < inf || delta > sup){
		//! esterno all'intervallo di controllo del servoRC
		printf("PWM esterno all'intervallo 5%% - 10%%\n");
		return;
	}
	else
		//! si imposta il delta del PWM ed il segnale esce dal pin
		switch(RCptr->numCHRc){
		case 1:
			RCptr->TIM_PWM->Instance->CCR1 = delta;
		break;

		case 2:
			RCptr->TIM_PWM->Instance->CCR2 = delta;
		break;

		case 3:
			RCptr->TIM_PWM->Instance->CCR3 = delta;
		break;

		case 4:
			RCptr->TIM_PWM->Instance->CCR4 = delta;
		break;

		default:
			printf("Numero del pwm errato\n");
		}

}
