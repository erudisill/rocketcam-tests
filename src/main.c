/*
 * main.c
 *
 *  Created on: Mar 31, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>
#include "cpdefs.h"



// JANKY - To keep from creating a .h for each test_xxx.c file
void init_clock(void);
void init_millis(void);
void init_led(void);
void init_uart0(void);
void init_sram(void);
extern volatile uint32_t millis;
extern CPSTRCONST uart_test;
extern CPSTRCONSTPTR volatile uart_test_ptr;



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
