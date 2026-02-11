/*
 * Slave_1.c
 * 
 * Created:
 * Author:
 */
//************************************************************************************
//================================== ESCLAVO 1 =====================================
//Este eslavo tiene la función de realizar la lectura para el MPU680
//*************************************************************************************
//Recordar que siempre hay que poner una resistencia pull-up 
// Para ver conexión (ver grabación clase 1:15:16)
// Para calcular valor (ver modulos de clase)

// Encabezado (librerías)
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "I2C_conf.h"
#include "ADC_Lib/ADC_lib.h"
#include "UART/UARTLib.h"

//Se define la dirección del esclavo, en este caso como es mi programa yo decido que dirección tiene
// caso contrario cuando se trabaja con un sensor, se debe de colocar la dirección descrita por el datasheet del sensor
#define SlaveAddress 0x30

uint16_t buffer = 0;
uint8_t dato_r [4]; 
uint8_t indice = 0; 
uint8_t bandera = 0; 
int16_t anguloX = 0;
int16_t anguloY = 0;

//************************************************************************************
// Function prototypes
void motor_steper(int16_t giro); 

//************************************************************************************
// Main Function
int main(void)
{
	
	DDRD |= (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5);
	PORTD &= ~((1 << PORTD2) | (1 << PORTD3) | (1 << PORTD4) | (1 << PORTD5));  
	DDRB |= (1 << DDB5);
	PORTB &= ~(1 << PORTB5); //Led para encender y apgar indicando una comunicación exitosa
	
	Servo2(0, 8); 
	//Servo3(no_invt);
	
	//inicializar ADC
	initADC(); 
	I2C_Slave_Init(0x30); //Se define la dirección del esclavo
	sei(); //Habilitar interrupciones
	
	while (1)
	{
		if(bandera){ //Reviso si el caractér de lectura esta recibiendose
			PINB |= (1 << PINB5); //Se hace un toggle para indicar que si hay datos 
			bandera = 0; 
			
			anguloX = (dato_r[0] << 8) | dato_r[1];
			anguloY = (dato_r[2] << 8) | dato_r[3]; 
			
		}
		//motor_steper((int16_t) anguloY);
		control_servo((int16_t) anguloX);
	}
}

//************************************************************************************
// NON-INterrupt subroutines

void motor_steper(int16_t angulo) // meter el valor del angulo de giro 
{
	if (angulo >= -10 && angulo <= 10) 
	{
		PORTD &= ~(1 << PORTD2);
		PORTD &= ~(1 << PORTD3);
		PORTD &= ~(1 << PORTD4);
		PORTD &= ~(1 << PORTD5); 
		_delay_ms(2);

	}
	
	if (angulo >= 10) // Dirección 1 
	{
		PORTD |= (1 << PORTD2);
		PORTD &= ~(1 << PORTD3);
		PORTD &= ~(1 << PORTD4);
		PORTD &= ~(1 << PORTD5);
		_delay_ms(2); 
		
		PORTD &= ~(1 << PORTD2);
		PORTD |= (1 << PORTD3);
		PORTD &= ~(1 << PORTD4);
		PORTD &= ~(1 << PORTD5);
		_delay_ms(2); 
		
		PORTD &= ~(1 << PORTD2);
		PORTD &= ~(1 << PORTD3);
		PORTD |= (1 << PORTD4);
		PORTD &= ~(1 << PORTD5);
		_delay_ms(2); 

		PORTD &= ~(1 << PORTD2);
		PORTD &= ~(1 << PORTD3);
		PORTD &= ~(1 << PORTD4);
		PORTD |= (1 << PORTD5);
		_delay_ms(2);
	}
	
	if (angulo <= -10) // Dirección 2
	{
		PORTD &= ~(1 << PORTD2);
		PORTD &= ~(1 << PORTD3);
		PORTD &= ~(1 << PORTD4);
		PORTD |= (1 << PORTD5);
		_delay_ms(2);
		
		PORTD &= ~(1 << PORTD2);
		PORTD &= ~(1 << PORTD3);
		PORTD |= (1 << PORTD4);
		PORTD &= ~(1 << PORTD5);
		_delay_ms(2);
		
		PORTD &= ~(1 << PORTD2);
		PORTD |= (1 << PORTD3);
		PORTD &= ~(1 << PORTD4);
		PORTD &= ~(1 << PORTD5);
		_delay_ms(2);

		PORTD |= (1 << PORTD2);
		PORTD &= ~(1 << PORTD3);
		PORTD &= ~(1 << PORTD4);
		PORTD &= ~(1 << PORTD5);
		_delay_ms(2);
	}
	
}

void control_servo(int16_t angulo)
{
	if (angulo > -15 && angulo < 15)
	{
		angulo = 0; // zona muerta
	}

	if (angulo < -90) angulo = -90;
	if (angulo > 90)  angulo = 90;

	uint16_t duty = 2000 + ((angulo + 90) * 2000) / 180;
	updateDutyCycle_servo2(duty);
}

//************************************************************************************
// Interrupt subroutines
ISR(TWI_vect){
	uint8_t estado = TWSR & 0xF8; // Máscara para el estado

	switch (estado) {
		case 0x60: // Master quiere escribirnos (SLA+W)
		indice = 0; // Reiniciamos el contador de bytes
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;

		case 0x80: // Dato recibido
		if (indice < 4) {
			dato_r[indice] = TWDR; // Guardamos el byte actual
			indice++;
		}
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;

		case 0xA0: // Se recibió un STOP o Repeated START
		bandera = 1; // Avisamos al main que ya terminó la trama
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;

		default:
		TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
		break;
	}

/*
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
		
	}*/
	
}