/* Copyright 2017-2018, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*==================[inlcusiones]============================================*/

// Includes de FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "task.h"
#include "semphr.h"

// sAPI header
#include "sapi.h"
#include "adaptacion_mpu9250.h"



/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

//DEBUG_PRINT_ENABLE;
SemaphoreHandle_t xSemaphoreTEC1F = NULL;
SemaphoreHandle_t xSemaphoreTEC1R = NULL;
SemaphoreHandle_t xSemaphoreD1 = NULL;
xQueueHandle Buffer, Seleccion;

uint32_t alfa=0;
uint8_t  beta=0;
//MPU9250_address_t addr = MPU9250_ADDRESS_0;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea

void initIRQ();
void GPIO0_IRQHandler(void);
void tareaA( void* taskParmPtr );
void tareaB( void* taskParmPtr );
void tareaC( void* taskParmPtr );
void tareaD( void* taskParmPtr );
void Debounce(void * taskParamPtr);

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma



   boardConfig();
   initIRQ();


   xSemaphoreTEC1F = xSemaphoreCreateBinary();
   xSemaphoreGive( xSemaphoreTEC1F );
   xSemaphoreTake( xSemaphoreTEC1F, ( TickType_t ) 10 );

   xSemaphoreTEC1R = xSemaphoreCreateBinary();
   xSemaphoreGive( xSemaphoreTEC1R );
   xSemaphoreTake( xSemaphoreTEC1R, ( TickType_t ) 10 );

   xSemaphoreD1 = xSemaphoreCreateBinary();
   xSemaphoreGive( xSemaphoreD1 );

   // UART for debug messages
   uartConfig( UART_USB, 115200 );
  // debugPrintConfigUart( UART_USB, 115200 );
  // debugPrintlnString( "Blinky con freeRTOS y sAPI." );



   // Led para dar se�al de vida
   gpioWrite( LED3, ON );
   gpioWrite( LED3, 0 );

   // Crear tarea en freeRTOS
//   xTaskCreate(
 //     myTask,                     // Funcion de la tarea a ejecutar
  //    (const char *)"myTask",     // Nombre de la tarea como String amigable para el usuario
   //   configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
    //  0,                          // Parametros de tarea
     // tskIDLE_PRIORITY+1,         // Prioridad de la tarea
     // 0                           // Puntero a la tarea creada en el sistema
 //  );

   xTaskCreate(
      tareaA,                     // Funcion de la tarea a ejecutar
      (const char *)"La tarea A",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      0,                          // Parametros de tarea
      tskIDLE_PRIORITY+0,         // Prioridad de la tarea
      0                           // Puntero a la tarea creada en el sistema
   );


   xTaskCreate(
         tareaB,                     // Funcion de la tarea a ejecutar
         (const char *)"Tarea BE !",     // Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
         0,                          // Parametros de tarea
         tskIDLE_PRIORITY+1,         // Prioridad de la tarea
         0                           // Puntero a la tarea creada en el sistema
      );

    xTaskCreate(
               tareaC,                     // Funcion de la tarea a ejecutar
               (const char *)"Tarea CE!",     // Nombre de la tarea como String amigable para el usuario
               configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
               0,                          // Parametros de tarea
               tskIDLE_PRIORITY+0,         // Prioridad de la tarea
              0                           // Puntero a la tarea creada en el sistema
           );


   xTaskCreate(
            tareaD,                     // Funcion de la tarea a ejecutar
            (const char *)"Tarea DE!",     // Nombre de la tarea como String amigable para el usuario
            configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
            0,                          // Parametros de tarea
            tskIDLE_PRIORITY+0,         // Prioridad de la tarea
            0                           // Puntero a la tarea creada en el sistema
         );
//
   xTaskCreate(
		    Debounce,                     // Funcion de la tarea a ejecutar
            (const char *)"Antirebote!!",     // Nombre de la tarea como String amigable para el usuario
            configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
            0,                          // Parametros de tarea
            tskIDLE_PRIORITY+0,         // Prioridad de la tarea
            0                           // Puntero a la tarea creada en el sistema
         );


   // Iniciar scheduler
   vTaskStartScheduler();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE ) {
      // Si cae en este while 1 significa que no pudo iniciar el scheduler
   }

   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion de la tarea
//void myTask( void* taskParmPtr )
//{
   // ---------- CONFIGURACIONES ------------------------------
   // ---------- REPETIR POR SIEMPRE --------------------------
	//while(TRUE) {
      // Intercambia el estado del LEDB
	   //if (beta){
	//if(alfa>0){
		   //alfa--;
		   // gpioWrite( LED1, 1 );
		   //   }
	   //else {		   gpioWrite( LED1, 0 );
	   // 	   beta=0;
	   	   	   	//	   }
//}
     // debugPrintlnString( "Blink!" );
      // Envia la tarea al estado bloqueado durante 500ms
//      vTaskDelay( 80 / portTICK_RATE_MS );
//   }
//}
//
void initIRQ() {
	Chip_PININT_Init(LPC_GPIO_PIN_INT);

	Chip_SCU_GPIOIntPinSel(0, 0, 4); //Mapeo del pin donde ocurrirá el evento y
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH0);//Se configura el canal
	Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH0);//Se configura para que el
	NVIC_SetPriority( PIN_INT0_IRQn, 5 );

	Chip_SCU_GPIOIntPinSel(1, 0, 4);//En este caso el canal de interrupción es 1
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH1);//En este caso el flanco es
	NVIC_SetPriority( PIN_INT1_IRQn, 5 );							       //de subida


	NVIC_EnableIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
}


void GPIO0_IRQHandler(void){
	portBASE_TYPE xSwitchRequired = pdFALSE;

	if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH0){ //Verificamos que la interrupción es la esperada
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0); //Borramos el flag de interrupción

		xSemaphoreGiveFromISR( xSemaphoreTEC1F, &xSwitchRequired); //En este caso libero un semáforo

	}
	portEND_SWITCHING_ISR(xSwitchRequired);//Terminar con taskYIELD_FROM_ISR (&xSwitchRequired);
}

void GPIO1_IRQHandler(void){
	portBASE_TYPE xSwitchRequired = pdFALSE;

	if (Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH1){
		Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
		xSemaphoreGiveFromISR( xSemaphoreTEC1R, &xSwitchRequired );
	}
	portEND_SWITCHING_ISR(xSwitchRequired);
}


//
void tareaA( void* taskParmPtr ){
	MPU9250_address_t addr = MPU9250_ADDRESS_0;
	int8_t status=0;
	float x=0,y=1,z=2;

	int i=0, d=6;
	uint8_t erre[d];
	uint8_t a=0;


	Seleccion=xQueueCreate( 2, sizeof( int ) );

	erre[d]=0;
	int len,j,temp, show;

	//void dacConfig( dacConfig_t config );
	//void dacWrite( dacMap_t analogOutput, uint16_t value );
	dacConfig( DAC_ENABLE );

	//unsigned char * chptr ;
	//chptr = (unsigned char*) malloc(4 * sizeof(char));

	int ipart =000;

	//uartWriteString( UART_USB, "Alfa!\r\n" );
	status = Ampu9250Init( addr );
	if(status){uartWriteString( UART_USB, "MPU conectado!\r\n" );
	}else{uartWriteString( UART_USB, "Mpu no detectado!\r\n" );	}
	//uartWriteString( UART_USB, "Beta!\r\n" );

	portTickType nett = 200 / portTICK_RATE_MS;

	while(TRUE) {
		portTickType xLastWakeTime = xTaskGetTickCount();



	Ampu9250Read();

	x=Ampu9250GetMagX_uT()*10;
	//Ampu9250GetAccelX_mss()*1000;
	//Ampu9250GetMagX_uT
    y=Ampu9250GetMagY_uT()*10;
    z=Ampu9250GetMagZ_uT()*10;

    x=x+250+11;
    y=y-490;
    z=z+211;


    ///////////

    	if(xSemaphoreTake( xSemaphoreD1, 1/portTICK_RATE_MS) == pdTRUE){
    		a++;
    	}

    	///////////


switch (a){
case 1:	show=x;			break;
case 2:	show=y;			break;
case 3:	show=z;			break;
default:show=x,a=1;		break;

};

	xQueueSend(Seleccion, ( void * ) &show, portMAX_DELAY);
    dacWrite( DAC, show); //


    ipart = (int) x;
    i=0;
    while (ipart>0)
      {
    erre[i++] =(ipart%10) + '0';
   	ipart = ipart/10;
     }

    while (i < d) {
    	erre[i++] = '0';}

    len=sizeof(erre);
    i=0;
    j=len-1;

    while (i<j)
    {
    	temp = erre[i];
    	erre[i] = erre[j];
    	erre[j] = temp;
    	i++; j--;
    }



    uartWriteString( UART_USB, "Giroscopo\r\n");

    uartWriteString( UART_USB, "uT*10 X:  ");
	uartWriteString( UART_USB, "\r\n" );
	erre[d]=0;
	uartWriteString( UART_USB, erre );
	uartWriteString( UART_USB, "\r\n" );

//
    ipart = (int)y;
    i=0;
    while (ipart>0)
      {
    erre[i++] =(ipart%10) + '0';
   	ipart = ipart/10;
     }

    while (i < d) {
    	erre[i++] = '0';}

    len=sizeof(erre);
    i=0;
    j=len-1;

    while (i<j)
    {
    	temp = erre[i];
    	erre[i] = erre[j];
    	erre[j] = temp;
    	i++; j--;
    }

//
    uartWriteString( UART_USB, "uT*10 Y:  ");
	uartWriteString( UART_USB, "\r\n" );
	erre[d]=0;
	uartWriteString( UART_USB, erre );
	uartWriteString( UART_USB, "\r\n" );
//
    ipart = (int)z;
    i=0;
    while (ipart>0)
      {

    erre[i++] =(ipart%10) + '0';
   	ipart = ipart/10;
     }

    while (i < d) {
    	erre[i++] = '0';}

    len=sizeof(erre);
    i=0;
    j=len-1;

    while (i<j)
    {
    	temp = erre[i];
    	erre[i] = erre[j];
    	erre[j] = temp;
    	i++; j--;
    }
//
    uartWriteString( UART_USB, "uT*10 Z:  ");
    uartWriteString( UART_USB, "\r\n" );
	erre[d]=0;
  	uartWriteString( UART_USB, erre );
   	uartWriteString( UART_USB, "\r\n\r\n" );

//
    ipart = show;
    i=0;
    while (ipart>0)
      {

    erre[i++] =(ipart%10) + '0';
   	ipart = ipart/10;
     }

    while (i < d) {
    	erre[i++] = '0';}

    len=sizeof(erre);
    i=0;
    j=len-1;

    while (i<j)
    {
    	temp = erre[i];
    	erre[i] = erre[j];
    	erre[j] = temp;
    	i++; j--;
    }

//

   	uartWriteString( UART_USB, "Mostrando:  ");
   	uartWriteString( UART_USB, "\r\n" );
   	erre[d]=0;
   	uartWriteString( UART_USB, erre );
   	uartWriteString( UART_USB, "\r\n\r\n" );



   	vTaskDelayUntil(&xLastWakeTime, nett);

	}
}

//
void tareaB( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
	Buffer=xQueueCreate( 3, sizeof( portTickType ) );
	portTickType LastFlankF=0/portTICK_RATE_MS;
	portTickType FlankF = xTaskGetTickCount();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {

				portTickType now = 39 / portTICK_RATE_MS;
		 		portTickType xLastWakeTime = xTaskGetTickCount();

	 		if ( xSemaphoreTake( xSemaphoreTEC1F, portMAX_DELAY) == pdTRUE) {	//Manejo de Flanco Bajada
	 								FlankF=xTaskGetTickCount();
	 								gpioWrite(LEDR,0);
	 								if((FlankF-LastFlankF)>(40/portTICK_RATE_MS))	//40 milisegundos del anterior Flanco
	 								{	xQueueSend(Buffer, ( void * ) &FlankF, portMAX_DELAY);
	 									//gpioWrite(LEDR,1);
	 								}
	 			 					//vTaskDelay(5/ portTICK_RATE_MS  );
	 			 					LastFlankF=FlankF;

	 			 				//	xSemaphoreGive( xSemaphore_Led_F );
	 			 				};

	 		vTaskDelayUntil(&xLastWakeTime, now);
	 		xLastWakeTime = xTaskGetTickCount();

	 		if ( xSemaphoreTake( xSemaphoreTEC1R, portMAX_DELAY) == pdTRUE) {	//Manejo de Flanco de Subida
	 							gpioWrite(LEDR,0);
	 						}
	 		vTaskDelayUntil(&xLastWakeTime, now);
	  //

   }
}
//
//									_____________
///				|--|  |--|	|--|	|
//40ms antes    |  |  |   |	|   |   |  40ms despues
//				|	|_|   |_	|  _|
//	____________|	 			|_|
//

void Debounce(void * taskParmPtr)	//Task creada para hacer antirrebote
{	uint8_t a=0;
	portTickType Rec=100;


	while (1) {

		if(Buffer != 0 ){
			if (xQueueReceive(Buffer, &Rec, 10/portTICK_RATE_MS)){
				a=1;
				}
			if((xTaskGetTickCount()-Rec)<(40/portTICK_RATE_MS)){
				//gpioWrite(LEDG,0);
				//gpioWrite(LEDR,1);
				}
			else{
				if(a){
				gpioWrite(LEDR,1);
				xSemaphoreGive( xSemaphoreD1 );
				//gpioWrite(LEDR,0);
				a=0;}
				}

		}
	}
}



//
void tareaC( void* taskParmPtr ){

	int b=0;
	while (1){
		if(Seleccion != 0 ){
						if (xQueuePeek(Seleccion, &b, portMAX_DELAY)){

								if(b<100){
								gpioWrite(LEDG,1);
								}
								else{
								gpioWrite(LEDG,0);
								}


						}
		}
	};
}

//
//
void tareaD( void* taskParmPtr ){
	int a=0;
	while (1){
		if(Seleccion != 0 ){
				if (xQueueReceive(Seleccion, &a, portMAX_DELAY)){


					///
					if (a==600){
						a=0;
					}

					if(a<60){
						gpioWrite(LED1,0);
						gpioWrite(LED2,0);
						gpioWrite(LED3,0);
					}
					else if(a<120){
						gpioWrite(LED1,1);
						gpioWrite(LED2,0);
						gpioWrite(LED3,0);
					}
					else if(a<180){
					gpioWrite(LED1,0);
					gpioWrite(LED2,1);
					gpioWrite(LED3,0);
					}
					//
					else if(a<240){
					gpioWrite(LED1,1);
					gpioWrite(LED2,1);
					gpioWrite(LED3,0);
					}
					else if(a<300){
					gpioWrite(LED1,0);
					gpioWrite(LED2,0);
					gpioWrite(LED3,1);
					}
					else if(a<360){
					gpioWrite(LED1,1);
					gpioWrite(LED2,0);
					gpioWrite(LED3,1);
					}
					else if(a<420){
					gpioWrite(LED1,0);
					gpioWrite(LED2,1);
					gpioWrite(LED3,1);
					}
					else if(a<880){
					gpioWrite(LED1,1);
					gpioWrite(LED2,1);
					gpioWrite(LED3,1);
					}
				}
		}
	};
}
//

/*==================[fin del archivo]========================================*/
