/*
 * main.c
 *
 *  Created on: Mar 31, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>

static volatile uint32_t millis = 0;

void init_clock(void) {
	// startup time value = 63
	// 15625us .. not sure where the number came from other than the max startup time
	// for the 3-20MHz Crystal Oscillator is 14.5ms in the datasheet (pg 1163.)
	// Likely, this number is left over from header files for other parts, and bigger is fine.
	// 15625us * (32000ticks / 1000000us) / 8 = 62.5
	// the "/8" is due to the fact that MOSCXTST is multiplied by 8
	//#define CLOCK_STARTUP_TIME_US	15625
	#define CLOCK_STARTUP_TIME_US	14500
	#define CLOCK_MOSCXTST			(CLOCK_STARTUP_TIME_US * (32000/1000000) / 8)

	// PLL values for 120MHz w/ 12MHz XTAL
	// (12MHz * 20) / 1 = 240MHz PLL
	#define CLOCK_PLL_MUL 			(20UL)
	#define CLOCK_PLL_DIV			(1UL)

	// 63 = settling time in us of PLLA, from datasheet pag 1166
	#define CLOCK_PLL_COUNT			(0x3fU)

	// master clock prescaler = 2 ... 240MHz PLL / 2 = 120MHz clock
	#define CLOCK_PRES				PMC_MCKR_PRES_CLK_2

	// Set flash wait state first or we won't be able to read code from memory!
	EFC0->EEFC_FMR = EEFC_FMR_FWS(5);	// 6 wait cycles .. datasheet pg 1199

	// Programming sequence from datasheet page 513

	//	2. Enable the fast crystal oscillator:
	PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
			CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTEN |
			CKGR_MOR_MOSCXTST(CLOCK_MOSCXTST);
	while (!(PMC->PMC_SR & PMC_SR_MOSCXTS));

	//	3. Switch MAIN CLOCK to crystal oscillator
	PMC->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCSEL;
	while (!(PMC->PMC_SR & PMC_SR_MOSCSELS));

	//	6. Set PLL and Divider
	PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0);		// disable first
	PMC->CKGR_PLLAR = CKGR_PLLAR_ONE |
						CKGR_PLLAR_MULA(CLOCK_PLL_MUL - 1) |
						CKGR_PLLAR_DIVA(CLOCK_PLL_DIV) |
						CKGR_PLLAR_PLLACOUNT(CLOCK_PLL_COUNT);
	while (!(PMC->PMC_SR & PMC_SR_LOCKA));

	//  7a. Select  Master Clock and Processor Clock
	PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | CLOCK_PRES;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY));

	//  7b. Set the master clock prescaler
	PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |	PMC_MCKR_CSS_PLLA_CLK;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
}


void TC0_Handler(void) {
	TC0->TC_CHANNEL[0].TC_SR;		// REQUIRED to clear the interrupt
	millis++;
}

int main(void) {

	// SETUP CLOCK TO 120MHz
	init_clock();

	// SETUP LED ON PIOC.P12
	PIOC->PIO_PUDR = PIO_PUDR_P12;		// disable pull-up
	PIOC->PIO_CODR = PIO_CODR_P12;		// set output data on pin 12 on PIOC
	PIOC->PIO_OER = PIO_OER_P12;		// enable output on pin 12 on PIOC
	PIOC->PIO_PER = PIO_PER_P12;		// enable pin 12 on PIOC

	// SETUP TIMER COUNTER
	PMC->PMC_PCER0 = (1 << ID_TC0);					// enable peripheral clock for TC0
	TcChannel * tcc = &TC0->TC_CHANNEL[0];			// Timer Counter 0, Channel 0
	tcc->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 |		// internal TIMER_CLOCK3 .. MCK/32
				 TC_CMR_CPCTRG;						// reset on RC compare match
	tcc->TC_RC = 3750;								// ms counter at 120MHz/32
	tcc->TC_IER = TC_IER_CPCS;						// enable interrupt on RC match
	tcc->TC_CCR = (TC_CCR_CLKEN | TC_CCR_SWTRG);	// enable and start

	// SETUP INTERRUPTS
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);

	uint32_t prev = 0;
	uint32_t elapsed = 0;

	while (1) {
		elapsed = millis - prev;
		if (elapsed >= 500) {
			if ((PIOC->PIO_ODSR & PIO_ODSR_P12) == 0) {
				PIOC->PIO_SODR = PIO_SODR_P12;
			} else {
				PIOC->PIO_CODR = PIO_SODR_P12;
			}
			prev = millis;
		}
	}
}
