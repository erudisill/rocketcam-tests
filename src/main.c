/*
 * main.c
 *
 *  Created on: Mar 31, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>

static volatile uint32_t millis = 0;
static char volatile received = 0;

// Requires .strings section in linker script, with KEEP so it isn't removed
// Placing the .strings section in RAM will provide quicker access but use up more memory.
// Placing the .strings section in ROM will maximize memory
#define CPSTRCONST			char * const __attribute((used, section(".strings")))
#define CPSTRCONSTPTR		char const *

static CPSTRCONST uart_test = "CPHANDHELD_";
static CPSTRCONSTPTR volatile uart_test_ptr;

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

void init_uart0(void) {
	// PIO: enable Peripheral A on URXD0 and UTXD0  (PA9 and PA10),  PID = 8
	PIOA->PIO_ABCDSR[0] &= ~(1 << ID_UART0);
	PIOA->PIO_ABCDSR[1] &= ~(1 << ID_UART0);
	PIOA->PIO_PDR = PIO_PA9A_URXD0 | PIO_PA10A_UTXD0;

	// PMC: enable Peripheral Clock for UART0 .. PID = 8
	PMC->PMC_PCER0 = (1 << ID_UART0);

	// NVIC: configure appropriate interrupts, before configuring UART
	NVIC_DisableIRQ(UART0_IRQn);
	NVIC_ClearPendingIRQ(UART0_IRQn);
	NVIC_SetPriority(UART0_IRQn, 0);
	NVIC_EnableIRQ(UART0_IRQn);

	// UART: set baud rate .. 38400  = 120MHz / (195 * 16)
	// UART: set baud rate .. 115200 = 120MHz / (65 * 16)
	UART0->UART_BRGR = 65;

	// UART: set no parity
	UART0->UART_MR = UART_MR_PAR_NO;

	// UART: start with TXRDY off and RXRDY on
	UART0->UART_IDR = UART_IDR_TXRDY;
	UART0->UART_IER = UART_IER_RXRDY;

	// UART: enable receiver and transmitter
	UART0->UART_CR = UART_CR_TXEN | UART_CR_RXEN;
}

void UART0_Handler(void) {
	uint32_t sr =  UART0->UART_SR;

	if (sr & UART_SR_TXRDY) {
		if (received) {
			UART0->UART_THR = received;
			received = 0;
		} else
			if (*uart_test_ptr) {
			UART0->UART_THR = *uart_test_ptr;
			uart_test_ptr++;
		} else {
			UART0->UART_IDR = UART_IDR_TXRDY;
		}
	}

	if (sr & UART_SR_RXRDY) {
		received = UART0->UART_RHR;
	}
}

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

void init_led(void) {
	// SETUP LED ON PIOC.P12
	PIOC->PIO_PUDR = PIO_PUDR_P12;		// disable pull-up
	PIOC->PIO_CODR = PIO_CODR_P12;		// set output data on pin 12 on PIOC
	PIOC->PIO_OER = PIO_OER_P12;		// enable output on pin 12 on PIOC
	PIOC->PIO_PER = PIO_PER_P12;		// enable pin 12 on PIOC
}

void init_sram(void) {
	uint32_t mask = 0;

	// Enable SMC peripheral (10)
	PMC->PMC_PCER0 = (1 << ID_SMC);

	// Configure SMC peripheral pins: data bus, address bus, NCS0, NRD, NWE
	// Note: Address bus is spread across PIOC and PIOA
	mask = 	0xFF 				// data bus D0-D7
			| PIO_PC8A_NWE 		// NWE
			| PIO_PC11A_NRD 	// NRD
			| PIO_PC14A_NCS0	// NCS0
			| 0xFFFC0000;			// address A0-A13
//			| 0xFFFD0000;			// address A0-A13 & A19  (A19?? Datasheet says this is A21 !! pg 52)
	PIOC->PIO_ABCDSR[0] &= ~mask;	// enable Peripheral A
	PIOC->PIO_ABCDSR[1] &= ~mask;	// enable Peripheral A
	PIOC->PIO_PUER = mask;			// enable pullup
	PIOC->PIO_PDR = mask;			// disable PIO control (enable Peripheral control)

//	mask = 0x001C0003;			// address A14-A18
	mask = 0x009C0003;			// address A14-A19
	PIOA->PIO_ABCDSR[0] &= ~mask;	// enable Peripheral C
	PIOA->PIO_ABCDSR[1] |= mask;	// enable Peripheral C
	PIOA->PIO_PUER = mask;			// enable pullup
	PIOA->PIO_PDR = mask;			// disable PIO control (enable Peripheral control)

	// Configure SRAM_VCC to turn on power to the chip
	PIOC->PIO_PUDR = PIO_PUDR_P9;	// disable pullup for PC9
	PIOC->PIO_CODR = PIO_CODR_P9;	// set output data on PC9 to 0
	PIOC->PIO_OER = PIO_OER_P9;		// enable output on PC9
	PIOC->PIO_PER = PIO_PER_P9;		// enable PIO control for PC9

	// NOTE: To turn off the SRAM_VCC, PC14 must be disabled and pulled down.
	//       Otherwise, it feeds the 3V_SRAM net and bypassess the MOSFET switch.
	//    PIOC->PIO_PER = PIO_PER_P14 ;
	//    PIOC->PIO_ODR = PIO_ODR_P14 ;
	//    PIOC->PIO_PUDR = PIO_PUDR_P14 ;
	//    PIOC->PIO_PPDER = PIO_PPDER_P14 ;


	// Configure SMC timing
	SMC->SMC_CS_NUMBER[0].SMC_SETUP = SMC_SETUP_NWE_SETUP( 2 )
									 | SMC_SETUP_NCS_WR_SETUP( 0 )
									 | SMC_SETUP_NRD_SETUP( 3 )
									 | SMC_SETUP_NCS_RD_SETUP( 0 ) ;

	SMC->SMC_CS_NUMBER[0].SMC_PULSE = SMC_PULSE_NWE_PULSE( 4 )
									 | SMC_PULSE_NCS_WR_PULSE( 5 )
									 | SMC_PULSE_NRD_PULSE( 4 )
									 | SMC_PULSE_NCS_RD_PULSE( 6 ) ;

	SMC->SMC_CS_NUMBER[0].SMC_CYCLE = SMC_CYCLE_NWE_CYCLE( 6 )
									 | SMC_CYCLE_NRD_CYCLE( 7 ) ;

	SMC->SMC_CS_NUMBER[0].SMC_MODE  = SMC_MODE_READ_MODE
									 | SMC_MODE_WRITE_MODE;


	// TEST TEST TEST TEST
	//

	#define BOARD_SRAM_BASE           ((void*)0x60000000)
	#define BOARD_SRAM_LENGTH         ((uint32_t)0x00100000)

	char * pSram = BOARD_SRAM_BASE;
	for (int i=0;i<0x100000;i++) {
		*(pSram + i) = 'A';
	}
	*(pSram + BOARD_SRAM_LENGTH - 4) = 'J';
	*(pSram + BOARD_SRAM_LENGTH - 3) = 'O';
	*(pSram + BOARD_SRAM_LENGTH - 2) = 'H';
	*(pSram + BOARD_SRAM_LENGTH - 1) = 'N';
	*(pSram + 0) = 'J';
	*(pSram + 1) = 'O';
	*(pSram + 2) = 'H';
	*(pSram + 3) = 'N';

}


int main(void) {

	// Disable Watchdog .. enabled at startup by default - ~16s
	WDT->WDT_MR = WDT_MR_WDDIS;

	init_clock();
	init_millis();
	init_led();
	init_uart0();
	init_sram();

	uint32_t prev = 0;
	uint32_t elapsed = 0;

	while (1) {
		elapsed = millis - prev;
		if (elapsed >= 500) {

			// toggle LED
			if ((PIOC->PIO_ODSR & PIO_ODSR_P12) == 0) {
				PIOC->PIO_SODR = PIO_SODR_P12;
			} else {
				PIOC->PIO_CODR = PIO_SODR_P12;
			}

			// transmit string when ready
			uart_test_ptr = uart_test;
			UART0->UART_IER = UART_IER_TXRDY;

			prev = millis;
		}
	}
}
