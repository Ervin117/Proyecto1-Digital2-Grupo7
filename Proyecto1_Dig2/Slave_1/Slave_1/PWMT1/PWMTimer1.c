/*
 * PWMTimer1.c
 *
 * Created: 4/22/2025 11:06:16 AM
 *  Author: razer
 */ 

#include "PWMTimer1.h"
void Servo2(uint8_t invertido, uint16_t presc) //cambiar nombres
{
	DDRB |= (1 << DDB1); // OC1A como salida

	
	TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0));

	if (invertido == invt)
	{
		TCCR1A |= (1 << COM1A1) | (1 << COM1A0); // modo invertido
	}
	else
	{
		TCCR1A |= (1 << COM1A1); // modo no invertido
	}

	TCCR1A |= (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12);

	ICR1 = 39999; //65000;//19999;

	switch (presc)
	{
		case 1:
		TCCR1B |= (1 << CS10);
		break;
		case 8:
		TCCR1B |= (1 << CS11);
		break;
		case 64:
		TCCR1B |= (1 << CS11) | (1 << CS10);
		break;
		case 256:
		TCCR1B |= (1 << CS12);
		break;
		case 1024:
		TCCR1B |= (1 << CS12) | (1 << CS10);
		break;
		default:
		TCCR1B |= (1 << CS11); // por defecto prescaler 8
		break;
	}
}
void updateDutyCycle_servo2(uint16_t duty) //cambiar nombres
{
	OCR1A = duty;
}

void Servo3(uint8_t invertido)
{
	DDRB |= (1 << DDB2); // OC1B como salida
	
	TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0)); // limpiar bits previos

	if (invertido == invt)
	{
		TCCR1A |= (1 << COM1B1) | (1 << COM1B0); // modo invertido
	}
	else
	{
		TCCR1A |= (1 << COM1B1); // modo no invertido
	}
}

// NUEVA FUNCIÓN: actualizar OCR1B
void updateDutyCycle_servo3(uint16_t duty1)
{
	OCR1B = duty1;
}