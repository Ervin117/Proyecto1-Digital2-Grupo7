/*
 * Programa_Maestro.c
 * 
 * Created:
 * Author: Ervin Gomez 
 * Description: Lectura del MPU6050 para control de un servo. 
 *
 */
//************************************************************************************
// Encabezado (librerías)
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <math.h> //Incluido para realiza operaciones matematicas
#include "I2C_conf.h"
#include "UART/UARTLib.h"

// --- DIRECCIONES I2C ---
#define MPU_ADDR 0x68
#define MPU_W (MPU_ADDR << 1)
#define MPU_R ((MPU_ADDR << 1) | 1)

// ----Dirección un esclvo----------
#define slave1 0x30
#define slave1R (0x30 << 1) | 0x01 //Pongo el último bit en 1 para lectura
#define slave1W (0x30 << 1) & 0b11111110 //Pongo el último bit en 0 para escribir

// --- REGISTROS ---
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B 

// --- VARIABLES GLOBALES PARA POSICIÓN ---
// Se declaran fuera para que guarden su valor entre ciclos
float vel_X = 0, vel_Y = 0, vel_Z = 0;
float pos_X = 0, pos_Y = 0, pos_Z = 0;


// --- PROTOTIPOS ---
void MPU6050_Init(void);
void angulo_giro(void);
void Enviar_Numero(int16_t numero); 
void Enviar_angulos(int16_t anguloX, int16_t anguloY); 

//************************************************************************************
int main(void)
{
    serialUART(); 
    I2C_Master_Init(100000, 1);
    
    _delay_ms(100);
    serialString("Iniciando MPU6050...\r\n");
    
    MPU6050_Init();
    
    // Calibración inicial simple (opcional):
    // Se asume que arranca quieto y plano.
    
    while (1)
    {
        // 1. Calcular y mostrar Ángulos
        angulo_giro();
		
        serialString("--------------------\r\n");

        PORTB ^= (1 << PORTB5); // Testigo visual
        _delay_ms(100); // Esto define el DT = 0.1 segundos
    }
}

// Función para calcular el angulo de giro
void angulo_giro(void)
{
    uint8_t high, low;
    int16_t Ax, Ay, Az;
    double ang_x, ang_y;

    // --- Lectura I2C ---
    I2C_Master_Start(); I2C_Master_Write(MPU_W); I2C_Master_Write(ACCEL_XOUT_H);
    I2C_Master_RepeatedStart(); I2C_Master_Write(MPU_R);
    
    I2C_Master_Read(&high, 1); I2C_Master_Read(&low, 1); Ax = (high << 8) | low;
    I2C_Master_Read(&high, 1); I2C_Master_Read(&low, 1); Ay = (high << 8) | low;
    I2C_Master_Read(&high, 1); I2C_Master_Read(&low, 0); Az = (high << 8) | low; 
    I2C_Master_Stop();

    // --- Matemáticas para hacer el calculo del angulo en los ejes
	ang_x = atan2(Ay, Az) * 57.296;
	ang_y = atan2(-Ax, Az) * 57.296;
	
    // Envio de datos por el serial para verificar funcionamienot
    serialString("ANGULO -> X: "); Enviar_Numero((int16_t)ang_x);
    serialString(" deg | Y: "); Enviar_Numero((int16_t)ang_y);
    serialString(" deg\r\n");
	Enviar_angulos((int16_t) ang_x, (int16_t) ang_y); 
	
}

void Enviar_angulos(int16_t anguloX, int16_t anguloY)
{
	 I2C_Master_Start();
	 
	 if(I2C_Master_Write(slave1W))
	 { 
		 /*
		 I2C_Master_Write(anguloX); // Byte 1: Angulo X
		 I2C_Master_Write(anguloY); // Byte 2: Angulo Y
		 I2C_Master_Stop();
		 */
		 I2C_Master_Write((uint8_t)(anguloX >> 8)); // byte alto
		 I2C_Master_Write((uint8_t)(anguloX & 0xFF)); // byte bajo

		 // Ángulo Y
		 I2C_Master_Write((uint8_t)(anguloY >> 8));
		 I2C_Master_Write((uint8_t)(anguloY & 0xFF));

		 I2C_Master_Stop();
	 }
	 else
	 {
		 // Si el esclavo no responde, liberar el bus para no colgar el programa
		 I2C_Master_Stop();
		 serialString("Error: Esclavo no responde\r\n");
	 }
}
// Funciones para inicar y enviar los numeros por serial 
void MPU6050_Init(void) {
    I2C_Master_Start(); I2C_Master_Write(MPU_W); 
    I2C_Master_Write(PWR_MGMT_1); I2C_Master_Write(0x00);
    I2C_Master_Stop(); _delay_ms(10);
}

void Enviar_Numero(int16_t numero) {
    char temp[7]; uint8_t i = 0;
    if (numero == 0) { serialLet('0'); return; }
    if (numero < 0) { serialLet('-'); numero = -numero; }
    while (numero > 0) { temp[i++] = (numero % 10) + '0'; numero /= 10; }
    while (i > 0) { serialLet(temp[--i]); }
}

//************************************************************************************
// Interrupt subroutines
ISR(USART_RX_vect) 
{
	//option = UDR0;
}

