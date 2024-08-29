/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MKL46Z4_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */
volatile unsigned int time_now_seconds = 0;
volatile unsigned int count = 0;
float sum_error = 0;

/*
 * @brief   Application entry point.
 */

//Button Interrupt Handler
int PID(){
	float level = 0;
	int error = 18 - count;
	float kp = 0.5;
	float ki = 0.1;
	float kd = 0.1;
	float dt = 0.25;
 	float integral_error = 0;
	integral_error = integral_error + (float)error*dt;
	float derivative_error = 0;
	derivative_error = derivative_error - error/dt;
	level = (kp*(float)error) + (ki*(float)integral_error) + (kd*(float)derivative_error);
	level = level * 100;
	int return_level = (int)level;

	return return_level;
}

void PORTC_PORTD_IRQHandler(void) { // Location defined in startup/startup_MKL46Z4.c
	PORTC->PCR[3] |= (1 << 24); // Clear Interrupt Flag!
	time_now_seconds = 0;
	while(1){
		if((time_now_seconds>=2) & (time_now_seconds<=7)){
			GPIOB->PDOR |= (1<<0);
			GPIOB->PDOR &= ~(1<<1);
			GPIOC->PDOR |= (1<<1);
			GPIOC->PDOR &= ~(1<<2);
		}else if(time_now_seconds>=8){
			GPIOB->PDOR &= ~(1<<0);
			GPIOB->PDOR &= ~(1<<1);
			GPIOC->PDOR &= ~(1<<1);
			GPIOC->PDOR &= ~(1<<2);
			break;
		}else{
			GPIOB->PDOR &= ~(1<<0);
			GPIOB->PDOR &= ~(1<<1);
			GPIOC->PDOR &= ~(1<<1);`
			GPIOC->PDOR &= ~(1<<2);
		}
	}
}
void setup_SW1_interrupt() {
	SIM->SCGC5 |= (1<<11);  // Enable Port C Clock
	PORTC->PCR[3] &= ~0xF0703; // Clear First
	PORTC->PCR[3] |= 0xF0703 & ((0xA << 16) | (1 << 8) | 0x3 ); // Set MUX bits, enable pullups, interrupt on falling edge
	GPIOC->PDDR &= ~(1 << 3); // Setup Pin 3 Port C as input


	// Leave as priority 0
	NVIC_SetPriority(31, 1);

	// Call Core API to Enable IRQ
	NVIC_EnableIRQ(31);
}

void PIT_IRQHandler(void){
	if(PIT->CHANNEL[0].TFLG){ // Timer 0 Triggered
		PIT->CHANNEL[0].TFLG = 1; // Reset
		time_now_seconds+=1;
	}
	if(PIT->CHANNEL[1].TFLG){
		PIT->CHANNEL[1].TFLG = 1;
		int temp = PID();
		if( ((TPM2->CONTROLS[0].CnV + temp) >= 7999) | ((TPM2->CONTROLS[1].CnV + temp) >= 7999)){//Ensures CNV does not go over 7999
			TPM2->CONTROLS[0].CnV = 7999;
			TPM2->CONTROLS[1].CnV = 7999;
		}else{
			TPM2->CONTROLS[0].CnV += temp;
			TPM2->CONTROLS[1].CnV += temp;
		}
		count = 0;

	}
}


void init_pits() {
	SIM->SCGC6 |= (1<<23);
	PIT->MCR = 0x00;
	PIT->CHANNEL[0].LDVAL = 24000000; // (1 seconds)
	PIT->CHANNEL[1].LDVAL = 4800000; // 0.2second
	NVIC_EnableIRQ(22);
	PIT->CHANNEL[0].TCTRL = 0x3; // enable Timer 0 interrupts and start timer.
	PIT->CHANNEL[1].TCTRL = 0x3; // Enable Timer 1 Interrupts and start timer


}

void init_encoder()
{
    SIM->SCGC5 |= (1<<9); //Enable clock gating
    PORTA->PCR[6] &= ~0xF0700;
    PORTA->PCR[6] |= 0xF0700 & ((0xA << 16)|(1 << 8));
    GPIOA->PDDR &= ~(1<<6); //Set as input
    NVIC_EnableIRQ(30);
}
void PORTA_IRQHandler(void)
{
    PORTA->PCR[6] |= (1<<24);
    count++;
}


int main(void) {

    /* Init board hardware. */
	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
    setup_SW1_interrupt();
    init_pits();
    init_encoder();

    //Motor Setup
    SIM->SCGC5 |= (1<<10) | (1<<11);

    //Left Motor Setup
    PORTB->PCR[0] &= ~0x700; //In1
    PORTB->PCR[0] |= 0x700 & (1<<8);

    PORTB->PCR[1] &= ~0x700;//In2
    PORTB->PCR[1] |= 0x700 & (1<<8);

    //Right Motor Setup
    PORTC->PCR[1] &= ~0x700;
    PORTC->PCR[1] |= 0x700 & (1<<8);

    PORTC->PCR[2] &= ~0x700;
    PORTC->PCR[2] |= 0x700 & (1<<8);

    //Enable Encoder pins as inputs
    PORTA->PCR[7] &= ~0x700; // Clear First
	PORTA->PCR[7] |= 0x700 & (1 << 8); // Set MUX bits
	PORTA->PCR[14] &= ~0x700; // Clear First
	PORTA->PCR[14] |= 0x700 & (1 << 8); // Set MUX bits
	PORTA->PCR[15] &= ~0x700; // Clear First
	PORTA->PCR[15] |= 0x700 & (1 << 8); // Set MUX bits

	//Enable encoder as inputs
	GPIOA->PDDR &= ~(1<<7);
	GPIOA->PDDR &= ~(1<<14);
	GPIOA->PDDR &= ~(1<<15);


    SIM->SCGC5 |= (1<<12);
    // Setup port for GPIO
   	PORTD->PCR[5] &= ~0x700; // Clear First
   	PORTD->PCR[5] |= 0x700 & (1 << 8); // Set MUX bits
   	GPIOD->PDDR |= (1 << 5); // Setup Pin 5 as output

   	//PWM Inputs
	PORTB->PCR[2] &= ~0x700;
	PORTB->PCR[2] |= 0x300;//Drive as PWM

	PORTB->PCR[3] &= ~0x700;
	PORTB->PCR[3] |= 0x300;//Drive as PWM



	GPIOB->PDDR |= (1<<0) | (1<<1);
	GPIOC->PDDR |= (1<<1)| (1<<2);

	//Setup for PWM on PTB2 (PWMA) Left Motor
	SIM->SCGC6 |= (1<<26);
	SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4); // Toggle Output on Match
	//Setup for PWM on PTB3 (PWMB) Right Motor
	TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4); // Toggle Output on Match
	TPM2->MOD = 7999;
	TPM2->CONTROLS[0].CnV = 1;
	TPM2->CONTROLS[1].CnV = 1;

	TPM2->SC |= 0x01 << 3;
	count = 0;
   	NVIC_EnableIRQ(30);
   	TPM2->CONTROLS[0].CnV = 5000;
   	TPM2->CONTROLS[1].CnV = 5000;
	while(1) {

	return 0 ;
	}
}
