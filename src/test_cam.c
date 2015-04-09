/*
 * test_cam.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>

static void init_parallel_capture(void) {
	// enable PIOA peripheral clock
	PMC->PMC_PCER0 = (1 << ID_PIOA);

	// disable parallel capture - distinct, single step
	// NOTE: This blows away prev config.
	PIOA->PIO_PCMR = ~((uint32_t)PIO_PCMR_PCEN);

	// disable receive buffer interrupts
	PIOA->PIO_PCIDR = PIO_PCIDR_RXBUFF;

	// 32-bit capture size, HSYNC/VSYNC BOTH required, FULL capture (not HALFS)
	// NOTE: This does NOT enable parallel capture
	PIOA->PIO_PCMR = PIO_PCMR_DSIZE_WORD;

//	// 8-bits a time, collect 4 bytes (32 bits) before data ready
//	PIOA->PIO_PCMR &= ~((uint32_t)PIO_PCMR_DSIZE_Msk);
//	PIOA->PIO_PCMR |= PIO_PCMR_DSIZE_WORD;
//
//	// start capture only when BOTH HSYNC and VSYNC are enabled
//	PIOA->PIO_PCMR &= ~((uint32_t)PIO_PCMR_ALWYS);
//
//	// collect every pulse (aka, color mode)
//	PIOA->PIO_PCMR &= ~((uint32_t)PIO_PCMR_HALFS);
}

static void init_twi(void) {
	// dwMck  = 120000000
	// dwTwCk = 100000
	// Trying to determine dwClDiv and dwCkDiv.
	// dwClDiv must be 8 bits, and dwCkDiv must be 3 bits
	// dwClDiv = ((dwMCk / (2 * dwTwCk)) - 4) / (1<<dwCkDiv)
	// 149    =            596                / (1<<2)
	// SO... for 100Kbs TWI on 120MHz CPU, Cl/ChDiv = 149 and CkDiv = 2
	//       for 400Kbs TWI on 120MHz CPU, Cl/ChDiv = 146 and CkDiv = 0
	// Note that in the datasheet (p.747), Tmck is the period, not freq. Meaning 1/120000000.
	// Then, double that number and it gives the total period in ns of the waveform.
	// The end result should equal 1/100000 or 1/400000 accordingly.
}

void init_cam(void) {

	// setup vsync/href interrupts

	// configure PIO Parallel Capture on PIOA
	init_parallel_capture();

	// turn on image sensor

	// configure programmable clock

	// configure twi

	// configure camera
}
