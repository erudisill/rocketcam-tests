/*
 * test_uart0.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */
#include <compiler.h>

// adapted from http://embedjournal.com/implementing-circular-buffer-embedded-c/
#define BUFFER_SIZE (80*25)
typedef struct {
	unsigned char buffer[BUFFER_SIZE];
	volatile unsigned int head;
	volatile unsigned int tail;
	volatile unsigned int count;
} ring_buffer;
static ring_buffer tx_buf = { { 0 }, 0, 0, 0 };
static ring_buffer rx_buf = { { 0 }, 0, 0, 0 };
bool uart0_putc(ring_buffer * buf, unsigned char data) {
	unsigned int next = (unsigned int) (buf->head + 1) % BUFFER_SIZE;
	if (next != buf->tail) {
		buf->buffer[buf->head] = data;
		buf->head = next;
		buf->count++;
		// turn on TXRDY interrupt if not already on
		if ((UART0->UART_IMR & UART_IMR_TXRDY) == 0) {
			UART0->UART_IER = UART_IER_TXRDY;
		}
		return true;
	} else {
		return false;
	}
}
bool uart0_getc(ring_buffer * buf, unsigned char * data) {
	if (buf->head == buf->tail) {
		buf->count = 0;
		return false;
	} else {
		*data = buf->buffer[buf->tail];
		buf->tail = (unsigned int) (buf->tail + 1) % BUFFER_SIZE;
		buf->count--;
		return true;
	}
}
bool uart0_readchar(unsigned char * data) {
	return uart0_getc(&rx_buf, data);
}
int uart0_get_tx_count(void) {
	return tx_buf.count;
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
	uint32_t sr = UART0->UART_SR;
	unsigned char c;

	if (sr & UART_SR_TXRDY) {
		if (uart0_getc(&tx_buf, &c)) {
			UART0->UART_THR = c;
		} else {
			UART0->UART_IDR = UART_IDR_TXRDY;
		}
	}

	if (sr & UART_SR_RXRDY) {
		uart0_putc(&rx_buf, UART0->UART_RHR);
	}
}

// newlib syscalls definitions
// put them here since they are piped to UART0

extern int _read(int file, char *ptr, int len) {
	int i;
	unsigned char c;
	for (i = 0; i < len; i++) {
		if (uart0_getc(&rx_buf, &c)) {
			ptr[i] = c;
		} else {
			break;
		}
	}
	return i;
}

extern int _write(int file, char *ptr, int len) {
	int i;

	// fill the ring buffer
	for (i = 0; i < len; i++) {
		if (uart0_putc(&tx_buf, ptr[i]) == false)
			break;
	}

	// return the number of chars successfully submitted
	// does this mean that newlib will buffer the writes?
	return i;
}

// OLD CODE OLD CODE OLD CODE
// OLD CODE OLD CODE OLD CODE
// OLD CODE OLD CODE OLD CODE
// OLD CODE OLD CODE OLD CODE

//#include "cpdefs.h"
//CPSTRCONST uart_test = "CPHANDHELD_";
//CPSTRCONSTPTR volatile uart_test_ptr;

//static char volatile received = 0;

//void UART0_Handler(void) {
//	uint32_t sr = UART0->UART_SR;
//
//	if (sr & UART_SR_TXRDY) {
//		if (received) {
//			UART0->UART_THR = received;
//			received = 0;
//		} else if (*uart_test_ptr) {
//			UART0->UART_THR = *uart_test_ptr;
//			uart_test_ptr++;
//		} else {
//			UART0->UART_IDR = UART_IDR_TXRDY;
//		}
//	}
//
//	if (sr & UART_SR_RXRDY) {
//		received = UART0->UART_RHR;
//	}
//}

//extern int _write(int file, char *ptr, int len) {
//	for (int i = 0; i < len; i++) {
//		UART0->UART_THR = ptr[i];
//		while (!(UART0->UART_SR & UART_SR_TXRDY))
//			;
//	}
//	return len;
//}
