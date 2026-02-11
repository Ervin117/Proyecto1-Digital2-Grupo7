/*
 * ADC_lib.c
 *
 * Created: 2/5/2026 7:51:30 PM
 *  Author: Ervin Gomez
 */ 
 #include "ADC_lib.h"
 
 
 void initADC()
 {
	 ADMUX = 0; //Se activa el canal 7.
	 ADMUX |= (1<<ADLAR);
	 ADMUX |= (1<<REFS0) | (1 << MUX0) | (1 << MUX1) |  (1 << MUX2); // | (1<<ADLAR)
	 
	 ADCSRA = 0;
	 ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2) | (1<<ADEN) | (1<<ADIE) ;
	 ADCSRA |= (1<<ADSC);
 }
 
 uint16_t lecADC(uint8_t canal)
 {
	 canal &= 0x03;
	 ADMUX &= 0xF8;
	 ADMUX |= canal;
	 ADCSRA |= (1<<ADSC); //Empezar conversión
	 while (ADCSRA & (1<<ADSC));
	 return ADC; //Leer los valores altos
 }