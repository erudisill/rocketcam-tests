/*
 * test_led.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>


void init_led(void) {
	// SETUP LED ON PIOC.P12
	PIOC->PIO_PUDR = PIO_PUDR_P12;		// disable pull-up
	PIOC->PIO_CODR = PIO_CODR_P12;		// set output data on pin 12 on PIOC
	PIOC->PIO_OER = PIO_OER_P12;		// enable output on pin 12 on PIOC
	PIOC->PIO_PER = PIO_PER_P12;		// enable pin 12 on PIOC
}
