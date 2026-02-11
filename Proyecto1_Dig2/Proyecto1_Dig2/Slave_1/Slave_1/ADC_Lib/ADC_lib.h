/*
 * ADC_lib.h
 *
 * Created: 2/5/2026 7:51:48 PM
 *  Author: razer
 */ 


#ifndef ADC_LIB_H_
#define ADC_LIB_H_
#define F_CPU 16000000
#include <avr/io.h>

void initADC(void);
uint16_t lecADC(uint8_t canal);



#endif /* ADC_LIB_H_ */