/*
 * PWMTimer1.h
 *
 * Created: 4/22/2025 11:06:32 AM
 *  Author: razer
 */ 

#ifndef PWMTIMER1_H_
#define PWMTIMER1_H_

#include <avr/io.h>
#define invt 1
#define no_invt 0

void Servo2(uint8_t invertido, uint16_t presc);
void updateDutyCycle_servo2(uint16_t duty); 

void Servo3(uint8_t invertido);
void updateDutyCycle_servo3(uint16_t duty1);

#endif /* PWMTIMER1_H_ */