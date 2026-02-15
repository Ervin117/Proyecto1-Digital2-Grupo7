/*
 * Slave2.c
 *
 * Created: 2/14/2026 10:01:33 PM
 * Author : Ervin Gomez
 * Configuración del sensor de color para el control de disparo 
 * del proyecto de Digital2
 */ 

// Encabezado (librerías)
#include <avr/io.h>
#include <stdint.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "I2C_Lib/I2C_conf.h"
#include "UART/UARTLib.h"


#define SlaveAddress 0x40 //Dirección del segundo esclavo para el sensor de color

//************************************************************************************
// Definición de variables
uint16_t buffer = 0; 
uint8_t colorDetectado = 0; 

//************************************************************************************
// Function prototypes
void leerColor(void); 

//************************************************************************************
// Main Function
int main(void)
{
	I2C_Slave_Init(0x40); 
	serialUART(); 
	serialString("Prueba para detectar color"); 
	
	DDRD |= (1<<DDD2) | (1<<DDD3) | (1<<DDD4) | (1<<DDD6); // S0,S1,S2,S3 salida
	DDRD &= ~(1<<DDD5); // D5 (T1) como entrada (OUT del sensor)

	// Escalamiento frecuencia al 20%
	PORTD |= (1<<PORTD2);   // S0 = 1
	PORTD &= ~(1<<PORTD3);  // S1 = 0

	// -------------------------
	// TIMER1 como contador externo
	// -------------------------
	TCCR1A = 0;
	TCCR1B = (1<<CS12) | (1<<CS11) | (1<<CS10);
	sei(); 
	while (1)
	{
		 leerColor();

		 if(colorDetectado == 1)
		 serialString("ROJO\r\n");
		 else if(colorDetectado == 2)
		 serialString("VERDE\r\n");
		 else if(colorDetectado == 3)
		 serialString("AZUL\r\n");
		 else
		 serialString("SIN COLOR\r\n");
		 _delay_ms(500);
		
	}
}

//************************************************************************************
// Interrupt subroutines
void leerColor(void)
{
	uint16_t rojo, verde, azul;

	// ================= ROJO =================
	PORTD &= ~(1<<PORTD4); // S2 = 0
	PORTD &= ~(1<<PORTD6); // S3 = 0

	TCNT1 = 0;
	_delay_ms(100);
	rojo = TCNT1;

	// ================= VERDE =================
	PORTD |= (1<<PORTD4);  // S2 = 1
	PORTD |= (1<<PORTD6);  // S3 = 1

	TCNT1 = 0;
	_delay_ms(100);
	verde = TCNT1;

	// ================= AZUL =================
	PORTD &= ~(1<<PORTD4); // S2 = 0
	PORTD |= (1<<PORTD6);  // S3 = 1

	TCNT1 = 0;
	_delay_ms(100);
	azul = TCNT1;

	// =============== COMPARACIÓN ===============
	if(rojo > verde && rojo > azul)
	colorDetectado = 1;
	else if(verde > rojo && verde > azul)
	colorDetectado = 2;
	else if(azul > rojo && azul > verde)
	colorDetectado = 3;
	else
	colorDetectado = 0; 
}

//************************************************************************************
// NON-INterrupt subroutines
ISR(TWI_vect)
{
uint8_t estado = TWSR & 0xFC; //Nos quedamos unicamente con los bits de estado TWI Status
	switch(estado){
		//**************************
		// Slave debe recibir dato
		//**************************
		case 0x60: //SLA+W recibido
		case 0x70: //General call
			TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE); //Indica "si te escuche"
			break;
		
		case 0x80: //Dato recibido, ACK enviado
		case 0x90: //Dato recibido General call, ACK enviado
			buffer = TWDR; //Ya puedo utilizar los datos // Dato que se utiliza para mover el servo 
			TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
			break;
			
		// Slave debe transmitir dato
		//*****************************
		//en cada case hay un comando que ya está predeterminado (ver presentacion)
		case 0xA8: //SLA+R recibido
		case 0xB8: //Dato transmitido, ACK recibido
			TWDR = buffer; //Dato a enviar
			TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
			break;
		//IMPORTANTE: que pasa si quiero enviar más de un dato?
		//Se puede hacer un arreglo por cada vez que envío un dato (ver grabación clase 1:08:11)
		
		case 0xC0: //Dato transmitido, NACK recibido
		case 0xC8: //Último dato transmitido
			TWCR = 0; //Limpio la interfaz para rebibir nuevo dato
			TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN); //Reiniciarse
			break;
			
		case 0xA0: // STOP o repeated START recibido como slave
			TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
			break;
		
		//**********************
		// Cualquier error
		//**********************
		default:
			TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
			break;
	}
}