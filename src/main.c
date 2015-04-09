/*
 * main.c
 *
 *  Created on: Mar 31, 2015
 *      Author: ericrudisill
 */

#include <inttypes.h>
#include <compiler.h>

// JANKY - To keep from creating a .h for each test_xxx.c file
void init_clock(void);
void init_millis(void);
void init_led(void);
void init_uart0(void);
bool uart0_readchar(unsigned char * data);
void init_sram(void);
bool test_sram_01(void);
void init_cam(void);
extern volatile uint32_t millis;

void print_menu(void) {
	printf("\r\n");
	printf("RocketCam Test Suite 1.0\r\n");
	printf("1. Toggle LED 1Hz\r\n");
	printf("2. Test SRAM\r\n");
}

int main(void) {
	uint32_t prev = 0;
	uint32_t elapsed = 0;
	uint32_t start = 0;
	bool result;
	bool led_on = false;
	unsigned char c;

	// Disable Watchdog .. enabled at startup by default - ~16s
	WDT->WDT_MR = WDT_MR_WDDIS;

	init_clock();
	init_millis();
	init_led();
	init_uart0();
	init_sram();
//	init_cam();


	print_menu();

	while (1) {

		if (uart0_readchar(&c)) {
			start = millis;
			if (c == '1') {
				led_on = (led_on ? false : true);
				printf("LED 1Hz is now ");
				(led_on ? puts("ON") : puts("OFF"));
			} else if (c == '2') {
				printf("Testing SRAM...\r\n");
				result = test_sram_01();
			} else {
				puts("UNKNOWN COMMAND");
				result = false;
			}
			elapsed = millis - start;
			(result ? printf("SUCCESS") : printf("FAILED"));
			printf(" ... elapsed: %" PRIu32 "\r\n", elapsed);
			print_menu();
		}


		if (led_on) {
			elapsed = millis - prev;
			if (elapsed >= 500) {
				// toggle LED
				if ((PIOC->PIO_ODSR & PIO_ODSR_P12) == 0) {
					PIOC->PIO_SODR = PIO_SODR_P12;
				} else {
					PIOC->PIO_CODR = PIO_SODR_P12;
				}
				prev = millis;
			}
		} else {
			PIOC->PIO_SODR = PIO_SODR_P12;	// led off
		}
	}
}
