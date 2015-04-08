/*
 * test_uart0.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */
#include <compiler.h>
#include "cpdefs.h"


CPSTRCONST uart_test = "CPHANDHELD_";
CPSTRCONSTPTR volatile uart_test_ptr;

static char volatile received = 0;



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
