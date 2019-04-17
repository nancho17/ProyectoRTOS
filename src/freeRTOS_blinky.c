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
#include "task.h"

// sAPI header
#include "sapi.h"
#include "adaptacion_mpu9250.h"



/*==================[definiciones y macros]==================================*/

/*==================[definiciones de datos internos]=========================*/

/*==================[definiciones de datos externos]=========================*/

//DEBUG_PRINT_ENABLE;

uint32_t alfa=0;
uint8_t  beta=0;
//MPU9250_address_t addr = MPU9250_ADDRESS_0;

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea
void myTask( void* taskParmPtr );
void myTask2( void* taskParmPtr );
void tareaA( void* taskParmPtr );

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{
   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();


   // UART for debug messages
   uartConfig( UART_USB, 115200 );
  // debugPrintConfigUart( UART_USB, 115200 );
  // debugPrintlnString( "Blinky con freeRTOS y sAPI." );



   // Led para dar se�al de vida
   gpioWrite( LED3, ON );

   // Crear tarea en freeRTOS
   xTaskCreate(
      myTask,                     // Funcion de la tarea a ejecutar
      (const char *)"myTask",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      0,                          // Parametros de tarea
      tskIDLE_PRIORITY+1,         // Prioridad de la tarea
      0                           // Puntero a la tarea creada en el sistema
   );

   xTaskCreate(
      myTask2,                     // Funcion de la tarea a ejecutar
      (const char *)"myTaskdos",     // Nombre de la tarea como String amigable para el usuario
      configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
      0,                          // Parametros de tarea
      tskIDLE_PRIORITY+1,         // Prioridad de la tarea
      0                           // Puntero a la tarea creada en el sistema
   );

   //
   xTaskCreate(
         tareaA,                     // Funcion de la tarea a ejecutar
         (const char *)"Tarea A",     // Nombre de la tarea como String amigable para el usuario
         configMINIMAL_STACK_SIZE*2, // Cantidad de stack de la tarea
         0,                          // Parametros de tarea
         tskIDLE_PRIORITY+0,         // Prioridad de la tarea
         0                           // Puntero a la tarea creada en el sistema
      );
   //

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
void myTask( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
      // Intercambia el estado del LEDB
if (beta){
	   if(alfa>0){
		   alfa--;
		   gpioWrite( LED1, 1 );
	   }
	   else {		   gpioWrite( LED1, 0 );
	   	   	   	   	   beta=0;
	   }
}
     // debugPrintlnString( "Blink!" );
      // Envia la tarea al estado bloqueado durante 500ms
      vTaskDelay( 80 / portTICK_RATE_MS );
   }
}
//
void myTask2( void* taskParmPtr )
{
   // ---------- CONFIGURACIONES ------------------------------
	int Actual=3;

	typedef enum{
	ESTADO_Rojo,
	ESTADO_RojoyAmarillo,
	ESTADO_Amarillo,
	ESTADO_Verde,
	} estadoMEF;

	typedef enum{
	Boton_presionado,
	Boton_bajando,
	Boton_nopresionado,
	Boton_subiendo,
	Boton,

	} MEFboton;

	gpioMap_t tecla = TEC1;

   // ---------- REPETIR POR SIEMPRE --------------------------
   while(TRUE) {
      // Intercambia el estado del LEDB

	   ///aswitcj
	         switch(Actual){
      case Boton_presionado:
    	  gpioWrite(LEDR,1);
    	  gpioWrite(LEDG,1);
    	  gpioWrite(LEDB,1);
    	  //cuenta
    	  alfa++;
    	  //
    	  if( gpioRead(tecla) ){
    		  Actual=Boton_subiendo;}
    	  break;

      case Boton_bajando:
    	  //delay(40);
		  if( !gpioRead(tecla) ){
			  Actual=Boton_presionado;}
		  else{
			  Actual=Boton_nopresionado;}
    	  break;

      case Boton_nopresionado:
      	  gpioWrite(LEDR,1);
      	  gpioWrite(LEDG,0);
      	  gpioWrite(LEDB,0);
      	  if(alfa>0){beta=1;}


      	  if( !gpioRead(tecla) ){
    		  Actual=Boton_bajando;}
      	  break;

      case Boton_subiendo:
    	  //delay(40);
    	  alfa++;
		  if( gpioRead(tecla) ){
			  Actual=Boton_nopresionado;}
		  else{
			  Actual=Boton_presionado;}

      	  break;
      default: 	 break;
      }
	   ///aswitcj
     // gpioToggle( LED1 );
     // debugPrintlnString( "Blink!" );
      // Envia la tarea al estado bloqueado durante 500ms
      vTaskDelay( 40 / portTICK_RATE_MS );
   }
}

//
void tareaA( void* taskParmPtr ){
	MPU9250_address_t addr = MPU9250_ADDRESS_0;
	int8_t status=0;
	float x=0,y=1,z=2;

	int i=0, d=6;
	uint8_t erre[d];
	erre[d]=0;
	int len,j,temp;


	//unsigned char * chptr ;
	//chptr = (unsigned char*) malloc(4 * sizeof(char));

	int ipart =000;

	uartWriteString( UART_USB, "Alfa!\r\n" );
	status = Ampu9250Init( addr );
	if(status){uartWriteString( UART_USB, "MPU conectado!\r\n" );
	}else{uartWriteString( UART_USB, "Mpu no detectado!\r\n" );	}
	uartWriteString( UART_USB, "Beta!\r\n" );

	while(TRUE) {


	Ampu9250Read();
	x=Ampu9250GetAccelX_mss()*1000;
    y=Ampu9250GetAccelY_mss()*1000;
    z=Ampu9250GetAccelZ_mss()*1000;



    uartWriteString( UART_USB, "gamma!\r\n" );

    ipart = (int)x;
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
    temp;
    while (i<j)
    {
    	temp = erre[i];
    	erre[i] = erre[j];
    	erre[j] = temp;
    	i++; j--;
    }



    uartWriteString( UART_USB, "yota!\r\n" );


    uartWriteString( UART_USB, "Giroscopo\r\n");

    uartWriteString( UART_USB, "GX:  ");
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
    temp;
    while (i<j)
    {
    	temp = erre[i];
    	erre[i] = erre[j];
    	erre[j] = temp;
    	i++; j--;
    }

//


    uartWriteString( UART_USB, "GY:  ");
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
    temp;
    while (i<j)
    {
    	temp = erre[i];
    	erre[i] = erre[j];
    	erre[j] = temp;
    	i++; j--;
    }






//

    uartWriteString( UART_USB, "GZ:  ");
    uartWriteString( UART_USB, "\r\n" );
	erre[d]=0;
  	uartWriteString( UART_USB, erre );
   	uartWriteString( UART_USB, "\r\n" );


	vTaskDelay( 1000 / portTICK_RATE_MS );




	}
}

//

//
/*==================[fin del archivo]========================================*/
