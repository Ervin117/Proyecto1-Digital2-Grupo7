/*
 * UARTLib.h
 *
 * Created: 1/26/2026 9:47:07 PM
 *  Author: razer
 */ 


#ifndef UARTLIB_H_
#define UARTLIB_H_

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


void serialUART(void); 
void serialLet(char letra); 
void serialString(char* oracion); 


#endif /* UARTLIB_H_ */