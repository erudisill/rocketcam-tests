/*
 * test_millis.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>


volatile uint32_t millis = 0;

void init_millis(void) {
	// SETUP TIMER COUNTER
	PMC->PMC_PCER0 = (1 << ID_TC0);			// enable peripheral clock for TC0
	TcChannel * tcc = &TC0->TC_CHANNEL[0];		// Timer Counter 0, Channel 0
	tcc->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 |	// internal TIMER_CLOCK3 .. MCK/32
			TC_CMR_CPCTRG;						// reset on RC compare match
	tcc->TC_RC = 3750;								// ms counter at 120MHz/32
	tcc->TC_IER = TC_IER_CPCS;					// enable interrupt on RC match
	tcc->TC_CCR = (TC_CCR_CLKEN | TC_CCR_SWTRG);	// enable and start

	// SETUP INTERRUPTS
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_EnableIRQ(TC0_IRQn);
}


void TC0_Handler(void) {
	TC0->TC_CHANNEL[0].TC_SR;		// REQUIRED to clear the interrupt
	millis++;
}
