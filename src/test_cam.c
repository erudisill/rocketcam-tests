/*
 * test_cam.c
 *
 *  Created on: Apr 8, 2015
 *      Author: ericrudisill
 */

#include <compiler.h>
#include "omnivision_ov7740.h"

#define BOARD_SRAM_BASE           ((void*)0x60000000)
#define IMAGE_WIDTH     (320*2)
#define IMAGE_HEIGHT    (240)

#define OV_CAPTOR_ADDRESS   (0x42>>1)   //OV7740 -> 0x42
#define	OV7740_PIDH               0x0a
#define	OV7740_PIDL               0x0b

extern volatile uint32_t millis;
extern int uart0_get_tx_count(void);

extern const ov_reg OV7740_QVGA_YUV422[];

volatile bool vsync_flag = false;
uint32_t volatile pc_count;

void Wait(uint32_t ms) {
	uint32_t start;
	start = millis;
	while ((millis - start) < ms)
		;
}

static inline uint8_t _clip( int32_t i )
{
    if ( i > 255 )
    {
        return 255 ;
    }

    if ( i < 0 )
    {
        return 0 ;
    }

    return (uint8_t)i ;
}

static void dump_buffer(void) {
	uint32_t dwCursor;
	int32_t C;
	int32_t D;
	int32_t E;
	int32_t dw1_1;
	int32_t dw1_2;
	int32_t dw1_3;
	uint8_t* pucData;
	uint8_t tempR;
	uint8_t tempG;
	uint8_t tempB;
	uint8_t step;

	pucData = (uint8_t *) BOARD_SRAM_BASE;

	printf("* START BUFFER *\r\n");
	while (uart0_get_tx_count() > 0);

	dwCursor = IMAGE_WIDTH * IMAGE_HEIGHT;
	step = 0;

	for (; dwCursor != 0; dwCursor -= 2, pucData += 4) {

		if (step == 0) {
			printf("\r\n%06X: ", dwCursor);
			while (uart0_get_tx_count() > 0);
			step = 8;
		}

		C = pucData[0]; // Y1
		C -= 16;
		D = pucData[3]; // U
		D -= 128;
		E = pucData[1]; // V
		E -= 128;

		dw1_1 = 516 * D + 128;
		dw1_2 = -100 * D - 208 * E + 128;
		dw1_3 = 409 * E + 128;

		tempB = _clip((298 * C + dw1_1) >> 8);
		tempG = _clip((298 * C + dw1_2) >> 8);
		tempR = _clip((298 * C + dw1_3) >> 8);
		printf("%02X,%02X,%02X ", tempB, tempG, tempR);

		C = pucData[2]; // Y2
		C -= 16;
		tempB = _clip((298 * C + dw1_1) >> 8);
		tempG = _clip((298 * C + dw1_2) >> 8);
		tempR = _clip((298 * C + dw1_3) >> 8);
		printf("%02X,%02X,%02X ", tempB, tempG, tempR);

		step--;
	}

	printf("* END BUFFER *\r\n");
}

static void _SetDefaultMaster(void) {
	Matrix *pMatrix = MATRIX;

	/* Set default master: SRAM (slave 0)-> Cortex-M4 System (Master 1)*/
	pMatrix->MATRIX_SCFG[0] |= ((1 << 18) & MATRIX_SCFG_FIXED_DEFMSTR_Msk) | /* Master 1 */
	((2 << 16) & MATRIX_SCFG_DEFMSTR_TYPE_Msk); /* Fixed Default Master */

	/* Set default master: Internal flash (slave 2) -> Cortex-M4 Instruction/Data (Master 0)*/
	pMatrix->MATRIX_SCFG[2] |= ((0 << 18) & MATRIX_SCFG_FIXED_DEFMSTR_Msk) | /* Master 0 */
	((2 << 16) & MATRIX_SCFG_DEFMSTR_TYPE_Msk); /* Fixed Default Master */

	/* Set default master: EBI (slave 3) -> PDC (Master 2)*/
	pMatrix->MATRIX_SCFG[3] |= ((2 << 18) & MATRIX_SCFG_FIXED_DEFMSTR_Msk) | /* Master 2 */
	((2 << 16) & MATRIX_SCFG_DEFMSTR_TYPE_Msk); /* Fixed Default Master */
}

#define TWITIMEOUTMAX 50000
uint8_t TWI_ByteSent(Twi *pTwi) {
	return ((pTwi->TWI_SR & TWI_SR_TXRDY) == TWI_SR_TXRDY);
}
void TWI_WriteByte(Twi *pTwi, uint8_t byte) {
	pTwi->TWI_THR = byte;
}
void TWI_StartWrite(Twi *pTwi, uint8_t address, uint32_t iaddress,
		uint8_t isize, uint8_t byte) {
	/* Set slave address and number of internal address bytes. */
	pTwi->TWI_MMR = 0;
	pTwi->TWI_MMR = (isize << 8) | (address << 16);

	/* Set internal address bytes. */
	pTwi->TWI_IADR = 0;
	pTwi->TWI_IADR = iaddress;

	/* Write first byte to send.*/
	TWI_WriteByte(pTwi, byte);
}
void TWI_SendSTOPCondition(Twi *pTwi) {
	pTwi->TWI_CR |= TWI_CR_STOP;
}
uint8_t TWI_TransferComplete(Twi *pTwi) {
	return ((pTwi->TWI_SR & TWI_SR_TXCOMP) == TWI_SR_TXCOMP);
}
static uint8_t twi_write(Twi * pTwi, uint8_t address, uint32_t iaddress,
		uint8_t isize, const uint8_t *pData, uint32_t num) {
	uint32_t timeout;

	// Start write
	TWI_StartWrite(pTwi, address, iaddress, isize, *pData++);
	num--;

	/* Send all bytes */
	while (num > 0) {

		/* Wait before sending the next byte */
		timeout = 0;
		while (!TWI_ByteSent(pTwi) && (++timeout < TWITIMEOUTMAX))
			;
		if (timeout == TWITIMEOUTMAX) {
			printf("TWID Timeout BS\n\r");
		}

		TWI_WriteByte(pTwi, *pData++);
		num--;
	}

	/* Wait for actual end of transfer */
	timeout = 0;

	/* Send a STOP condition */
	TWI_SendSTOPCondition(pTwi);

	while (!TWI_TransferComplete(pTwi) && (++timeout < TWITIMEOUTMAX))
		;
	if (timeout == TWITIMEOUTMAX) {
		printf("TWID Timeout TC2\n\r");
	}

	return 0;
}

static uint8_t twi_read(Twi * pTwi, uint8_t address, uint32_t iaddress,
		uint8_t isize, uint8_t *pData, uint32_t num) {

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
		while (((pTwi->TWI_SR & TWI_SR_RXRDY) != TWI_SR_RXRDY)
				&& (++timeout < TWITIMEOUTMAX))
			;
		if (timeout == TWITIMEOUTMAX) {
			printf("TWI Timeout BR\n\r");
		}
		*pData++ = pTwi->TWI_RHR;
		num--;
	}

	/* Wait for transfer to be complete */
	timeout = 0;
	while (((pTwi->TWI_SR & TWI_SR_TXCOMP) != TWI_SR_TXCOMP)
			&& (++timeout < TWITIMEOUTMAX))
		;
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
//	TWI0->TWI_CWGR = TWI_CWGR_CKDIV(2) | TWI_CWGR_CHDIV(149) | TWI_CWGR_CLDIV(149);		// 100KHz
	TWI0->TWI_CWGR = TWI_CWGR_CKDIV(
			0) | TWI_CWGR_CHDIV(146) | TWI_CWGR_CLDIV(146);		// 400KHz
}


/**
 *  Initialize a list of OV registers.
 *  The list of registers is terminated by the pair of values
 *  { OV_REG_TERM, OV_VAL_TERM }.
 *  Returns zero if successful, or non-zero otherwise.
 *  \param pTwi TWI interface
 *  \param pReglist Register list to be written
 *  \return 0 if no error, otherwize TWID_ERROR_BUSY
 */
int ov_write_regs(Twi *pTwi, const ov_reg* pReglist) {
	int err;
	int size = 0;
	const ov_reg *pNext = pReglist;

	printf("ov_write_regs:");

	while (!((pNext->reg == 0xFF) && (pNext->val == 0xFF))) {
		if (pNext->reg == 0xFE) {
			Wait(5);
		} else {
			//err = ov_write_reg( pTwid, pNext->reg, pNext->val ) ;
			err = twi_write(TWI0, OV_CAPTOR_ADDRESS, pNext->reg, 1, &pNext->val,
					1);
			printf("+(%d) ", size);
			size++;

			if (err != 0) {
				printf("ov_write_regs: TWI ERROR\n\r");

				return err;
			}
		}
		pNext++;
	}

	printf("\n\r");

	return 0;
}

extern void PIOA_Handler(void) {
	uint32_t sr = PIOA->PIO_ISR;

	if ((sr & PIO_ISR_P15) != 0) {
		vsync_flag = true;
	}

}

static void init_vsync(void) {
	PIOA->PIO_IDR = PIO_IER_P15;	// disable interrupt during configuration
	PIOA->PIO_PUER = PIO_PUER_P15;		// enable pullup for PA15
	PIOA->PIO_ODR = PIO_OER_P15;		// enable input for PA15
	PIOA->PIO_AIMER = PIO_AIMER_P15;	// enable additional interrupt modes
	PIOA->PIO_ESR = PIO_ESR_P15;		// edge event detection
	PIOA->PIO_REHLSR = PIO_REHLSR_P15;	// rising edge event detection
	PIOA->PIO_PER = PIO_PER_P15;		// enable PIO control for PA15

	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 0);
	NVIC_EnableIRQ(PIOA_IRQn);
}

static void init_parallel_capture(void) {
	// enable PIOA peripheral clock
	PMC->PMC_PCER0 = (1 << ID_PIOA);

	// disable parallel capture - distinct, single step
	// NOTE: This blows away prev config.
	PIOA->PIO_PCMR = ~((uint32_t) PIO_PCMR_PCEN);

	// disable receive buffer interrupts
	PIOA->PIO_PCIDR = PIO_PCIDR_RXBUFF;

	// 32-bit capture size, HSYNC/VSYNC BOTH required, FULL capture (not HALFS)
	// NOTE: This does NOT enable parallel capture
	PIOA->PIO_PCMR = PIO_PCMR_DSIZE_WORD;
}

void init_cam(void) {

	// set default values for PDC
	_SetDefaultMaster();

	// setup vsync interrupts
	init_vsync();

	// configure PIO Parallel Capture on PIOA
	init_parallel_capture();

	// turn on image sensor power switch
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
	while (!(PMC->PMC_SCSR & PMC_SCSR_PCK0))
		;

	// configure twi
	init_twi();
}

void PIO_Capture_Switch(Pio *pio, bool on_off) {
	if (on_off) {
		pio->PIO_PCMR |= PIO_PCMR_PCEN;
	} else {
		pio->PIO_PCMR &= (~(uint32_t) PIO_PCMR_PCEN);
	}
}

uint8_t PIO_CaptureToBuffer(Pio *pio, uint8_t *buf, uint32_t size) {
	/* check if the first PDC bank is free*/
	if ((pio->PIO_RCR == 0) && (pio->PIO_RNCR == 0)) {
		pio->PIO_RPR = (uint32_t) buf;
		pio->PIO_RCR = size;
		pio->PIO_PTCR = PIO_PTCR_RXTEN;

		return 1;
	} else if (pio->PIO_RNCR == 0) {

		pio->PIO_RNPR = (uint32_t) buf;
		pio->PIO_RNCR = size;
		return 1;
	} else {
		return 0;
	}
}

bool PIO_Capture_BUFF(Pio *pio) {
	return ((pio->PIO_PCISR & PIO_PCIMR_RXBUFF) == PIO_PCIMR_RXBUFF) ?
	true :
																		false;
}

bool test_cam_capture(void) {

	uint8_t *buf;

	buf = (uint8_t *) BOARD_SRAM_BASE;

	// sync with vsync
	vsync_flag = false;
	PIOA->PIO_ISR;
	PIOA->PIO_IER = PIO_IER_P15;
	while (!vsync_flag)
		;
	PIOA->PIO_IDR = PIO_IDR_P15;

	// turn on parallel capture
	PIO_Capture_Switch(PIOA, true);

	// capture to buffer
	PIO_CaptureToBuffer(PIOA, buf, (IMAGE_HEIGHT * IMAGE_WIDTH) >> 2);
	while (!PIO_Capture_BUFF(PIOA))
		;

	// turn off parallel capture
	PIO_Capture_Switch(PIOA, false);

	// Dump buffer to UART
	dump_buffer();
}

//bool test_cam_capture(void) {
//
//	uint32_t pcsr = PIOA->PIO_PCISR;
//	uint8_t *buf;
//
//	buf = (uint8_t *) BOARD_SRAM_BASE;
//
//	// sync with vsync
//	vsync_flag = false;
//	PIOA->PIO_ISR;
//	PIOA->PIO_IER = PIO_IER_P15;
//	while (!vsync_flag)
//		;
//	PIOA->PIO_IDR = PIO_IDR_P15;
//
//	// Capture 100 bytes
//	pc_count = 100;
//	PIOA->PIO_PCMR |= PIO_PCMR_PCEN;
//	pcsr = PIOA->PIO_PCISR;
//	while (pc_count > 0) {
//		pcsr = PIOA->PIO_PCISR;
//		if ((pcsr & PIO_PCISR_DRDY)) {
//			buf[100 - pc_count] = PIOA->PIO_PCRHR;
//			pc_count--;
//		}
//	}
//	PIOA->PIO_PCMR &= (~(uint32_t) PIO_PCMR_PCEN);
//}

bool test_cam_twi(void) {
	// dwStatus = TWID_Read( pTwid, OV_CAPTOR_ADDRESS, ucReg, 1, pucVal, 1, 0 ) ;

	uint8_t id = 0;
	uint8_t ver = 0;
	uint8_t status;

	status = twi_read(TWI0, OV_CAPTOR_ADDRESS, OV7740_PIDH, 1, &id, 1);
	status = twi_read(TWI0, OV_CAPTOR_ADDRESS, OV7740_PIDL, 1, &ver, 1);
	printf("PID = 0x%X\r\n", id);
	printf("VER = 0x%X\r\n", ver);

	// configure camera
	ov_write_regs(TWI0, OV7740_QVGA_YUV422);

	return true;
}

