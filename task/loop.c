/*
 * loop.c
 *
 *  Created on: 07 feb 2019
 *      Author: massimo
 */

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include "servomotoreRC.h"
#include <stdio.h>
#include <inttypes.h>
#include "main.h"

extern volatile uint32_t TICK, MS100, S1;
extern servoRC RC[];
extern uint32_t buffer[];
extern bool ADupdate;
extern ADC_HandleTypeDef hadc3;
#define		MS_500		50

volatile int JY1_X;
volatile int JY1_Y;
volatile int JY1_SW;
volatile int JY2_X;
volatile int JY2_Y;
volatile int JY2_SW;

volatile bool state1 = true;
volatile bool state2 = false;

volatile bool button1_old = true;
volatile bool button2_old = true;

#define min 0.040

volatile float PWM_base = 0.075;
volatile float PWM_spalla = 0.075;
volatile float PWM_gomito = 0.075;
volatile float PWM_polso = 0.075;
volatile float PWM_mano = 0.075;
volatile float PWM_pinza = 0.075;

volatile float MAX_base = 0.130;
volatile float MAX_spalla = 0.110;
volatile float MAX_gomito = 0.127;
volatile float MAX_polso = 0.1245;
volatile float MAX_mano = 0.128;
volatile float MAX_pinza = 0.073;

// memoria
volatile float pos[200][6];
volatile int mem = 0;
volatile bool button3 = false;
volatile bool button3_old = true;

volatile bool programmation = false;
volatile bool execution = false;

volatile bool exit_old;
volatile bool exit_new;

int first_run = true;
uint32_t T_500 = 0;

void loop(void){

	if ((TICK - T_500) >= MS_500){
	///lampeggio ogni 500 ms.
	HAL_GPIO_TogglePin(GPIOB, LD1_Pin|LD2_Pin|LD3_Pin);
		T_500 = TICK;
	}

	// verifica se occorre aggiornare le letture dei convertitori
	if (ADupdate == false)
		/// aggiorna ogni volta che il dato e' usato, la lettura dei convertitori AD
		/// La funzione callback riporta ADupdate a true
		HAL_ADC_Start_DMA(&hadc3, buffer, 6);
	/*for(float i=0; i < 0.140; i=i+0.001){
	RC[1].delta = (uint32_t) RC[1].periodo *i;
	goRC(&RC[1]);

	printf("%d\n", RC[1].delta);
	}*/

	if(execution == false){

		JY1_X = buffer[0]; //base
		JY1_Y = buffer[1]; //spalla
		JY1_SW= buffer[2]; //joystick sinistro
		JY2_X = buffer[3]; //mano //gomito
		JY2_Y = buffer[4]; //polso //pinza
		JY2_SW= buffer[5]; //joystick destro

		/* Metodo vecchio di lettura del pulsante
	 	int a = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10); //viene letto JY1_SW
		if( button2_old == false && a == GPIO_PIN_RESET ){ //JY1_SW � stato premuto?
			button2_old = true;
			if( state1 == true ){ //si, si entra nello stato di movimento della pinza
				state1 = false; //esce dallo stato 1
				state2 = true; //entra nello stato 2
			}
			else{ //no, rimane nello stato di movimento dei primi quattro motori
				state2 = false; //non entra nello stato 2
				state1 = true; //rimane nello stato 1
			}
		}
		else
			button2_old = false;*/

		int a = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10);            //viene letto JY1_SW
		if( button2_old == false && a == true ){      //JY1_SW � stato premuto?
			if( state1 == true ){							   //si, si entra nello stato di movimento della pinza
				state1 = false;                               //esce dallo stato 1
				state2 = true;                               //entra nello stato 2
			}
			else{                                          //no, rimane nello stato di movimento dei primi quattro motori
				state2 = false;                           //non entra nello stato 2
				state1 = true;                           //rimane nello stato 1
			}
		}
		button2_old = a;


		button3_old = button3;
		button3 = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);     //Viene letto il pulsante 3
		if(button3 == true && button3_old == false){       //� stato premuto il pulsante 3?
			if(programmation == true){					  //Siamo nella fase di programmazione?
				programmation = false;                   //Si, esci dalla fase di programmazione
				execution = true;						//Si, entra in quella di esecuzione
			}
			else if(execution == true){            //No, ma siamo nella fase di esecuzione?
				programmation = true;             //Si, entra nella fase di programmazione
				execution = false;               //Si, esci dalla fase di programmazione
			}
		}

		if( state1 == true){                                    //Siamo nella fase 1?
			if( JY1_X > 3000 && PWM_base < MAX_base )           //Se
				PWM_base = PWM_base + 0.001;
			//printf("%d" PRIu32 "\n", PWM_base);
			if( JY1_X < 1500 && PWM_base > min )
				PWM_base = PWM_base - 0.001;
			if( JY1_Y > 3000 && PWM_spalla > min )
				PWM_spalla = PWM_spalla - 0.001;
			if( JY1_Y < 1500 && PWM_spalla < MAX_spalla )
				PWM_spalla = PWM_spalla + 0.001;
			if( JY2_X > 3500 && PWM_gomito < MAX_gomito )
				PWM_gomito = PWM_gomito + 0.001;
			if( JY2_X < 1500 && PWM_gomito > min )
				PWM_gomito = PWM_gomito - 0.001;
			if( JY2_Y > 3500 && PWM_mano > min )
				PWM_mano = PWM_mano - 0.001;
			if( JY2_Y < 1500 && PWM_mano < MAX_mano )
				PWM_mano = PWM_mano + 0.001;

			HAL_Delay(45);    //pausa di 45 ms tra i vari incrementi

		}
		else if( state2 == true){                                //Siamo nello stato 2?
																//Si
			if( JY2_X > 3500 && PWM_pinza < MAX_pinza )
				PWM_pinza = PWM_pinza + 0.001;
			if( JY2_X < 1500 && PWM_pinza > min )
				PWM_pinza = PWM_pinza - 0.001;
			if( JY2_Y > 3500 && PWM_polso > min )
				PWM_polso = PWM_polso - 0.001;
			if( JY2_Y < 1500 && PWM_polso < MAX_polso )
				PWM_polso = PWM_polso + 0.001;

			HAL_Delay(30);   //pausa di 30 ms tra i vari incrementi
		}


		if(programmation == true){       //Siamo nella fase di prgrammazione?
			pos[mem][0] = PWM_base;      //Si, salva questa posizione del primo motore
			pos[mem][2] = PWM_gomito;   //Si, salva questa posizione del terzo motore
			pos[mem][1] = PWM_spalla;  //Si, salva questa posizione del secondo motore
			pos[mem][3] = PWM_mano;   	//Si, salva questa posizione del quarto motore
			pos[mem][4] = PWM_pinza;    //Si, salva questa posizione del quinto motore
			pos[mem][5] = PWM_polso;   //Si, salva questa posizione della pinza
		}

		RC[0].delta = (uint32_t) RC[0].periodo * PWM_base;
		goRC(&RC[0]);                                                //muove la base
		RC[1].delta = (uint32_t) RC[1].periodo * PWM_spalla;
		goRC(&RC[1]);                                               //muove la spalla
		RC[2].delta = (uint32_t) RC[2].periodo * PWM_gomito;
		goRC(&RC[2]);                                              //muove il gomito
		RC[3].delta = (uint32_t) RC[3].periodo * PWM_mano;
		goRC(&RC[3]);                                             //muove la mano
		RC[4].delta = (uint32_t) RC[4].periodo * PWM_polso;
		goRC(&RC[4]);                                            //muove il polso
		RC[5].delta = (uint32_t) RC[5].periodo * PWM_pinza;
		goRC(&RC[5]);                                           //muove la pinza

		if(mem >= 200 && programmation == true){               //abbiamo raggiunto 200 posizioni memorizzate?
			execution = true;
			state1 = false;
			state2 = false;
		}

		else if(programmation == true)
			mem = mem + 1;
	}


	else if(execution == true){                                            //esegue le posizioni memorizzate al contrario
		for(int d = (mem - 1); d >= 0; d--){
			RC[0].delta = (uint32_t) RC[0].periodo * pos[d][0];
			goRC(&RC[0]);
			RC[1].delta = (uint32_t) RC[1].periodo * pos[d][2];
			goRC(&RC[1]);
			RC[2].delta = (uint32_t) RC[2].periodo * pos[d][1];
			goRC(&RC[2]);
			RC[3].delta = (uint32_t) RC[3].periodo * pos[d][3];
			goRC(&RC[3]);
			RC[4].delta = (uint32_t) RC[4].periodo * pos[d][4];
			goRC(&RC[4]);
			RC[5].delta = (uint32_t) RC[5].periodo * pos[d][5];
			goRC(&RC[5]);

			HAL_Delay(30);
		}
		for(int d = 0; d <= (mem - 1); d++){                                //esegue le posizioni memorizzate
			RC[0].delta = (uint32_t) RC[0].periodo * pos[d][0];
			goRC(&RC[0]);
			RC[1].delta = (uint32_t) RC[1].periodo * pos[d][2];
			goRC(&RC[1]);
			RC[2].delta = (uint32_t) RC[2].periodo * pos[d][1];
			goRC(&RC[2]);
			RC[3].delta = (uint32_t) RC[3].periodo * pos[d][3];
			goRC(&RC[3]);
			RC[4].delta = (uint32_t) RC[4].periodo * pos[d][4];
			goRC(&RC[4]);
			RC[5].delta = (uint32_t) RC[5].periodo * pos[d][5];
			goRC(&RC[5]);

			HAL_Delay(30);
		}
		exit_old = exit_new;
		exit_new = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);         //Il pulsante rosso � stato premuto?
		if(exit_new == true && exit_old == false){
			execution = false;								   //Esce dalla fase di esecuzione
			state1 = true;                                    //Entra nello stato 1
			mem = 0;                                         //Azzera il numero delle posizioni memorizzate
			for(int d = 0; d <= mem -1; d++){               //Azzera le posizioni memorizzate
				pos[d][0] = ' ';
				pos[d][2] = ' ';
				pos[d][1] = ' ';
				pos[d][3] = ' ';
				pos[d][4] = ' ';
				pos[d][5] = ' ';
			}
		}
	}
}

/**
  * @brief 	Inizializza i 6 motori RC con i valori di posizionamento pari a 90° cioe' a meta della corsa
  * @param 	nessuno.
  * @retval nessuno
  */
void setup(){
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

	  RC[0].delta = (uint32_t) RC[0].periodo *0.075;
	  RC[1].delta = (uint32_t) RC[1].periodo *0.075;
	  RC[2].delta = (uint32_t) RC[2].periodo *0.075;
	  RC[3].delta = (uint32_t) RC[3].periodo *0.075;
	  RC[4].delta = (uint32_t) RC[4].periodo *0.075;
	  RC[5].delta = (uint32_t) RC[5].periodo *0.075;

	  for (int i = 0; i < 6; i++)
	  	 //! le strutture dati sono impostate e i PWM vengono avviati
	  	 goRC(&RC[i]);

}
