/*
 * test_clock.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>

//#define CP_CLOCK

#ifdef CP_CLOCK

// startup time value = 63
// 15625us .. not sure where the number came from other than the max startup time
// for the 3-20MHz Crystal Oscillator is 14.5ms in the datasheet (pg 1163.)
// Likely, this number is left over from header files for other parts, and bigger is fine.
// 15625us * (32000ticks / 1000000us) / 8 = 62.5
// the "/8" is due to the fact that MOSCXTST is multiplied by 8
//#define CLOCK_STARTUP_TIME_US	15625
#define CLOCK_STARTUP_TIME_US	14500
#define CLOCK_MOSCXTST			(CLOCK_STARTUP_TIME_US * (32000/1000000) / 8)

// PLL A values for 120MHz w/ 12MHz XTAL
// (12MHz * 20) / 1 = 240MHz PLL
#define CLOCK_PLLA_MUL 			(20UL)
#define CLOCK_PLLA_DIV			(1UL)

// PLL B values for 96MHz w/ 12MHz XTAL
// (12MHz * 8) / 1 = 96MHz PLL
// Later, 96MHz will be prescaled by 4 to create a 24MHz clock for the camera
#define CLOCK_PLLB_MUL 			(8UL)
#define CLOCK_PLLB_DIV			(1UL)


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

	//	6a. Set PLL and Divider - for MCK
	PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0);		// disable first
	PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(CLOCK_PLLA_MUL - 1) |
						CKGR_PLLAR_DIVA(CLOCK_PLLA_DIV) |
						CKGR_PLLAR_PLLACOUNT(CLOCK_PLL_COUNT);
	while (!(PMC->PMC_SR & PMC_SR_LOCKA))
		;

	//	6b. Set PLL and Divider - for Camera (PCK0)
	PMC->CKGR_PLLBR = CKGR_PLLBR_MULB(0);		// disable first
	PMC->CKGR_PLLBR = CKGR_PLLBR_MULB(CLOCK_PLLB_MUL - 1) |
						CKGR_PLLBR_DIVB(CLOCK_PLLB_DIV) |
						CKGR_PLLBR_PLLBCOUNT(CLOCK_PLL_COUNT);
	while (!(PMC->PMC_SR & PMC_SR_LOCKB))
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

#else

#define BOARD_MCK 120000000

/* Clock settings at 48MHz */
#if (BOARD_MCK == 48000000)
#define	NUMBER_WS	2
#define BOARD_OSCOUNT   (CKGR_MOR_MOSCXTST(0x8))
#define BOARD_PLLBR     (CKGR_PLLBR_MULB(3) \
                       | CKGR_PLLBR_PLLBCOUNT(0x1) \
                       | CKGR_PLLBR_DIVB(1))
#define BOARD_PLLAR     (CKGR_PLLAR_STUCKTO1 | CKGR_PLLAR_MULA(7) \
                       | CKGR_PLLAR_PLLACOUNT(0x1) \
                       | CKGR_PLLAR_DIVA(1))
#define BOARD_MCKR      (PMC_MCKR_PRES_CLK | PMC_MCKR_CSS_PLLB_CLK)
/* Clock settings at 64MHz */
#elif (BOARD_MCK == 64000000)
#define	NUMBER_WS	3
#define BOARD_OSCOUNT   (CKGR_MOR_MOSCXTST(0x8))
#define BOARD_PLLBR     (CKGR_PLLBR_MULB(31) \
                       | CKGR_PLLBR_PLLBCOUNT(0x1) \
                       | CKGR_PLLBR_DIVB(3))
#define BOARD_PLLAR     (CKGR_PLLAR_STUCKTO1 | CKGR_PLLAR_MULA(7) \
                       | CKGR_PLLAR_PLLACOUNT(0x1) \
                       | CKGR_PLLAR_DIVA(1))
#define BOARD_MCKR      (PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLB_CLK)
/* Clock settings at 120MHz */
#elif (BOARD_MCK == 120000000)
#define	NUMBER_WS	5
#define BOARD_OSCOUNT   (CKGR_MOR_MOSCXTST(0x8))
#define BOARD_PLLBR     (CKGR_PLLBR_MULB(29) \
                       | CKGR_PLLBR_PLLBCOUNT(0x1) \
                       | CKGR_PLLBR_DIVB(3))
#define BOARD_PLLAR     (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(7) \
                       | CKGR_PLLAR_PLLACOUNT(0x1) \
                       | CKGR_PLLAR_DIVA(1))
#define BOARD_MCKR      (PMC_MCKR_PRES_CLK_1 | PMC_MCKR_CSS_PLLB_CLK)
#else
    #error "No settings for current BOARD_MCK."
#endif

/* Define clock timeout */
#define CLOCK_TIMEOUT    0xFFFFFFFF

/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Performs the low-level initialization of the chip.
 * This includes EFC and master clock configuration.
 * It also enable a low level on the pin NRST triggers a user reset.
 */
void init_clock( void )
{
    uint32_t timeout = 0;

    /* Set 5 FWS for Embedded Flash Access
    and CLOE: Code Loops Optimization Enable
    128-bit flash access */
    EFC0->EEFC_FMR = (1<<26) | EEFC_FMR_FWS(NUMBER_WS);

    /* Initialize main oscillator */
    if ( !(PMC->CKGR_MOR & CKGR_MOR_MOSCSEL) )
    {
        PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN;
        timeout = 0;
        while (!(PMC->PMC_SR & PMC_SR_MOSCXTS) && (timeout++ < CLOCK_TIMEOUT));
    }

    /* Switch to 3-20MHz Xtal oscillator */
    PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCSEL;
    timeout = 0;
    while (!(PMC->PMC_SR & PMC_SR_MOSCSELS) && (timeout++ < CLOCK_TIMEOUT));
    PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK;
    for ( timeout = 0; !(PMC->PMC_SR & PMC_SR_MCKRDY) && (timeout++ < CLOCK_TIMEOUT) ; );

    /* Initialize PLLB for CPU and Bus clock */
    PMC->CKGR_PLLBR = BOARD_PLLBR;
    timeout = 0;
    while (!(PMC->PMC_SR & PMC_SR_LOCKB) && (timeout++ < CLOCK_TIMEOUT));

    /* Switch to main clock */
    PMC->PMC_MCKR = (BOARD_MCKR & ~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK;
    for ( timeout = 0; !(PMC->PMC_SR & PMC_SR_MCKRDY) && (timeout++ < CLOCK_TIMEOUT) ; );

    PMC->PMC_MCKR = BOARD_MCKR ;
    for ( timeout = 0; !(PMC->PMC_SR & PMC_SR_MCKRDY) && (timeout++ < CLOCK_TIMEOUT) ; );

    /* Initialize PLLA for image sensor */
    PMC->CKGR_PLLAR = BOARD_PLLAR;
    timeout = 0;
    while (!(PMC->PMC_SR & PMC_SR_LOCKA) && (timeout++ < CLOCK_TIMEOUT));
}

#endif
