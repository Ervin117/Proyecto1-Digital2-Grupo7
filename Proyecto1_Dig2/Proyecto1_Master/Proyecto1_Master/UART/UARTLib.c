/*
 * UARTLib.c
 *
 * Created: 1/26/2026 9:46:49 PM
 *  Author: razer
 */ 

#include "UARTLib.h"

void serialUART()
{
	DDRD |= (1<< DDD1);
	DDRD &= ~(1<<DDD0);
	
	//configurado para enviar y recibir.
	UCSR0A = 0;
	UCSR0B |= (1<< RXCIE0) | (1<<RXEN0) | (1<<TXEN0);
	UCSR0C |= (1<< UCSZ01) | (1<<UCSZ00);
	UBRR0 = 103; //valor calculado para los 9600
}

void serialLet(char letra)
{
	while ((UCSR0A & (1<<UDRE0)) == 0);
	UDR0 = letra;
	
}

void serialString(char* oracion)
{
	for (uint8_t i = 0; *(oracion+i) != '\0'; i++)
	{
		serialLet(*(oracion+i));
	}
}