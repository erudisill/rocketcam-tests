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


////////
// HACK: Ripped from pseudo-ASF files in WPIR demo project.  Should use a proper driver!!
////////
static uint8_t twi_read(Twi * pTwi, uint8_t address, uint32_t iaddress, uint8_t isize, uint8_t *pData, uint32_t num) {

	#define TWITIMEOUTMAX 50000

    uint32_t timeout;

    /* Set STOP signal if only one byte is sent*/
    if (num == 1) {
        pTwi->TWI_CR = TWI_CR_STOP;
    }

    /* Set slave address and number of internal address bytes. */
    pTwi->TWI_MMR = 0;
    pTwi->TWI_MMR = (isize << 8) | TWI_MMR_MREAD | (address << 16);

    /* Set internal address bytes */
    pTwi->TWI_IADR = 0;
    pTwi->TWI_IADR = iaddress;

    /* Send START condition */
    pTwi->TWI_CR = TWI_CR_START;


    /* Read all bytes, setting STOP before the last byte*/
    while (num > 0) {

        /* Last byte ?*/
        if (num == 1) {
            pTwi->TWI_CR = TWI_CR_STOP;
        }

        /* Wait for byte then read and store it*/
        timeout = 0;
        while( ((pTwi->TWI_SR & TWI_SR_RXRDY) != TWI_SR_RXRDY) && (++timeout<TWITIMEOUTMAX) );
        if (timeout == TWITIMEOUTMAX) {
            printf("TWI Timeout BR\n\r");
        }
        *pData++ = pTwi->TWI_RHR;
        num--;
    }

    /* Wait for transfer to be complete */
    timeout = 0;
    while( ((pTwi->TWI_SR & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) && (++timeout<TWITIMEOUTMAX) );
    if (timeout == TWITIMEOUTMAX) {
        printf("TWI Timeout TC\n\r");
    }

    return 0;
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

	// configure TWI pins (Peripheral A)
	PIOA->PIO_ABCDSR[0] &= ~(1 << ID_TWI0);
	PIOA->PIO_ABCDSR[1] &= ~(1 << ID_TWI0);
	PIOA->PIO_PDR = PIO_PA4A_TWCK0 | PIO_PA3A_TWD0;

	// enable TWI peripheral clock
	PMC->PMC_PCER0 = (1 << ID_TWI0);

	// configure TWI interrupts
	NVIC_DisableIRQ(TWI0_IRQn);
	NVIC_ClearPendingIRQ(TWI0_IRQn);
	NVIC_SetPriority(TWI0_IRQn, 0);
	NVIC_EnableIRQ(TWI0_IRQn);

	// Slave Enabled and Reset
	TWI0->TWI_CR = TWI_CR_SVEN;
	TWI0->TWI_CR = TWI_CR_SWRST;
	TWI0->TWI_RHR;

	// Disable Slave and Master
	TWI0->TWI_CR = TWI_CR_SVDIS;
	TWI0->TWI_CR = TWI_CR_MSDIS;

	// NOW, enable Master mode
	TWI0->TWI_CR = TWI_CR_MSEN;

	// Configure waveform (see notes above)
	TWI0->TWI_CWGR = 0;
	TWI0->TWI_CWGR = TWI_CWGR_CKDIV(2) | TWI_CWGR_CHDIV(149) | TWI_CWGR_CLDIV(149);
}

void init_cam(void) {
	// setup vsync/href interrupts

	// configure PIO Parallel Capture on PIOA
	init_parallel_capture();

	// turn on image sensor
	PIOC->PIO_PUDR = PIO_PUDR_P10;		// disable pullup for PC10
	PIOC->PIO_CODR = PIO_CODR_P10;		// set output data on PC10 to 0
	PIOC->PIO_OER = PIO_OER_P10;		// enable output on PC10
	PIOC->PIO_PER = PIO_PER_P10;		// enable PIO control for PC10


	// configure programmable clock - PCK0 on PB13, PERIPH B
	unsigned int abcdsr;
	unsigned int mask;
	mask = PIO_PB13B_PCK0;

	PIOB->PIO_IDR = mask;
	PIOB->PIO_PUDR = mask;

	abcdsr = PIOB->PIO_ABCDSR[0];
	PIOB->PIO_ABCDSR[0] = (mask | abcdsr);
	abcdsr = PIOB->PIO_ABCDSR[1];
	PIOB->PIO_ABCDSR[1] &= (~mask & abcdsr);
	PIOB->PIO_PDR = mask;

	PMC->PMC_PCK[0] = PMC_PCK_CSS_PLLB_CLK | PMC_PCK_PRES_CLK_4;
	PMC->PMC_SCER = PMC_SCER_PCK0;
	while (!(PMC->PMC_SCSR & PMC_SCSR_PCK0)) ;

	// configure twi
	init_twi();

	// configure camera
}

bool test_cam_twi(void) {
	// dwStatus = TWID_Read( pTwid, OV_CAPTOR_ADDRESS, ucReg, 1, pucVal, 1, 0 ) ;
	#define OV_CAPTOR_ADDRESS   (0x42>>1)   //OV7740 -> 0x42
	#define	OV7740_PIDH               0x0a
	#define	OV7740_PIDL               0x0b

    uint8_t id=0;
    uint8_t ver=0;
    uint8_t status;

    status = twi_read(TWI0, OV_CAPTOR_ADDRESS, OV7740_PIDH, 1, &id, 1);
    status = twi_read(TWI0, OV_CAPTOR_ADDRESS, OV7740_PIDL, 1, &ver, 1);
    printf("PID = 0x%X\r\n", id);
    printf("VER = 0x%X\r\n", ver);

    return true;
}
