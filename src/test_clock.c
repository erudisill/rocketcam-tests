/*
 * test_clock.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>


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




void init_clock(void) {
	// Set flash wait state first or we won't be able to read code from memory!
	EFC0->EEFC_FMR = EEFC_FMR_FWS(5);// 6 wait cycles for 120MHz .. datasheet pg 1199

	// Programming sequence from datasheet page 513

	//	2. Enable the fast crystal oscillator:
	PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
	CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCXTST(CLOCK_MOSCXTST);
	while (!(PMC->PMC_SR & PMC_SR_MOSCXTS))
		;

	//	3. Switch MAIN CLOCK to crystal oscillator
	PMC->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCSEL;
	while (!(PMC->PMC_SR & PMC_SR_MOSCSELS))
		;

	//	6. Set PLL and Divider
	PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0);		// disable first
	PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(CLOCK_PLL_MUL - 1) |
	CKGR_PLLAR_DIVA(CLOCK_PLL_DIV) |
	CKGR_PLLAR_PLLACOUNT(CLOCK_PLL_COUNT);
	while (!(PMC->PMC_SR & PMC_SR_LOCKA))
		;

	//  7a. Select  Master Clock and Processor Clock
	PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | CLOCK_PRES;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY))
		;

	//  7b. Set the master clock prescaler
	PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk))
			| PMC_MCKR_CSS_PLLA_CLK;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY))
		;
}

