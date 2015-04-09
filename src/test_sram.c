/*
 * test_sram.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>

#define BOARD_SRAM_BASE           ((void*)0x60000000)
#define BOARD_SRAM_LENGTH         ((uint32_t)0x00100000)

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

}

bool test_sram_01(void) {
	char * pSram = BOARD_SRAM_BASE;
	for (int i=0;i<0x100000;i++) {
		*(pSram + i) = 'A';
	}

	// Write to the mid point
	*(pSram + (BOARD_SRAM_LENGTH/2) - 4) = 'C';
	*(pSram + (BOARD_SRAM_LENGTH/2) - 3) = 'P';
	*(pSram + (BOARD_SRAM_LENGTH/2) - 2) = 'H';
	*(pSram + (BOARD_SRAM_LENGTH/2) - 1) = 'T';

	// Write to the max point
	*(pSram + BOARD_SRAM_LENGTH - 4) = 'T';
	*(pSram + BOARD_SRAM_LENGTH - 3) = 'H';
	*(pSram + BOARD_SRAM_LENGTH - 2) = 'P';
	*(pSram + BOARD_SRAM_LENGTH - 1) = 'C';

	printf("test_sram_01: bytes at mid should be 'CPHT' are actually '");
	putchar((uint8_t)*(pSram + (BOARD_SRAM_LENGTH/2) - 4));
	putchar((uint8_t)*(pSram + (BOARD_SRAM_LENGTH/2) - 3));
	putchar((uint8_t)*(pSram + (BOARD_SRAM_LENGTH/2) - 2));
	putchar((uint8_t)*(pSram + (BOARD_SRAM_LENGTH/2) - 1));
	printf("\r\n");
	printf("test_sram_01: bytes at end should be 'THPC' are actually '");
	putchar((uint8_t)*(pSram + BOARD_SRAM_LENGTH - 4));
	putchar((uint8_t)*(pSram + BOARD_SRAM_LENGTH - 3));
	putchar((uint8_t)*(pSram + BOARD_SRAM_LENGTH - 2));
	putchar((uint8_t)*(pSram + BOARD_SRAM_LENGTH - 1));
	printf("\r\n");

	uint8_t test = 0;
	test |= *(pSram + (BOARD_SRAM_LENGTH/2) - 4) ^ 'C';
	test |= *(pSram + (BOARD_SRAM_LENGTH/2) - 3) ^ 'P';
	test |= *(pSram + (BOARD_SRAM_LENGTH/2) - 2) ^ 'H';
	test |= *(pSram + (BOARD_SRAM_LENGTH/2) - 1) ^ 'T';
	test |= *(pSram + BOARD_SRAM_LENGTH - 4) ^ 'T';
	test |= *(pSram + BOARD_SRAM_LENGTH - 3) ^ 'H';
	test |= *(pSram + BOARD_SRAM_LENGTH - 2) ^ 'P';
	test |= *(pSram + BOARD_SRAM_LENGTH - 1) ^ 'C';

	printf("test_sram_01: test result is %#02x\r\n", test);

	return (test == 0 ? true : false);
}
