
/*
 */

/**
 * @mainpage
 *
 * \image html logo_big.png
 * - Main project site: http://usb-ir-remote.sourceforge.net
 * - Releases: http://sourceforge.net/projects/usb-ir-remote/files/
 * - GIT: git://usb-ir-remote.git.sourceforge.net/gitroot/usb-ir-remote/usb-ir-remote (read-only)
 * .
 *
 * \section intro Introduction
 * This project includes a USB device that receives Ir light from a remote control (Sony and NEC protocol)
 * and report the keys pressed on the remote control to the PC using the standardized USB HID device.
 * There is s special class for remote controls and this firmware will register as such a device.
 *
 * Main features:
 * - All source code and hardware design available
 * - Very low cost
 * - All development tools are free, including the hardware and programmer to flash the chip
 * - Can be extended to other uses
 * - Device is programmable to accept different types of remotes
 * .
 *
 * \section dev_environment Development Environment
 * I started to develop on Windows (http://winavr.sourceforge.net). I now moved to Linux.
 * Tools (for Ubuntu Linux):
 * - AVR GCC ("$ apt-get install avr-gcc")
 * - Eagle CAD (http://www.cadsoftusa.com)
 * - AVR Dude ("$ apt-get install avrdude")
 * - Doxygen (http://www.doxygen.org "$apt-get install doxygen")
 * .
 * \section user_guide User Guide
 *
 * Welcome to the USB Ir Remote Device. When plugged into the computer
 * the device will register it self as an USB Consumer HID device.
 * It will act as a keyboard and a consumer device that can send
 * play, stop, pause, etc commands to applications.
 *
 * Windows Media Player have built-in support for these commands and the HID
 * driver is included Windows. No drivers are needed! No girder or WinLIRC
 * or similar is needed.
 *
 * The device will react to NEC/SONY compatible IR codes. The NEC protocols is
 * used by a number of manufacturers including Pioneer, Onkyo and Goldstar.
 *
 * To learn the codes for the different commands do this:
 * - Open a notepad windows (Start->Run->"notepad")
 * - Plug-in the device
 * - Press the program button on the device
 * - "IR Keyboard..." should be printed in Notepad
 * - "Press button" will appear and then the button to press
 * - Press each button requested by the device
 * - "DONE" will appear when all programmable ir buttons have been programmed.
 * .
 *
 * Your computer should now be ready to control Windows Media Player.
 * Start your favorite movie and try the start, stop and pause buttons on the
 * remote.
 *
 * \section implementation_guide Implementation
 * - \ref software
 * - \ref hardware
 * .
 *
 * (c) 2007 Jorgen Birkler (jorgen.birkler)a(gmail.com)
 *
 * USB driver
 *
 * (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 */
/**
 * \page software Software
 * Uses the firmware only USB low speed driver from http://obdev.at.
 * The USB device is configured as a Remote Control HID device.
 *
 * \section tips Tips about HID development
 * General tips about HID development:
 *
 * 1. HID device class is cached by Windows; change USB_CFG_DEVICE_ID if you change USAGE_PAGE
 *    class to another. It took me several weeks to find this info. I copied the use page for the
 *    remote but it never work until I changed the USB_CFG_DEVICE_ID to another number so that the
 *    device was rediscovered by Windows.
 *
 * 2. Added usbconfig.h manually to the dependencies in the make file to all .o files.
 *    WinAVR .d files doesn't seem to work for subdirs
 *
 * Ir is received by ICP interrupt:
 * \include irrx.h
 *
 * Main loop translated the ir codes received and handles the main USB look:
 * \include main.c
 */
/**
 * \page hardware Hardware
 *
 * Schematic:
 * \image html ir_hid.sch.png Schematic
 *
 * Partlist:
 * \verbinclude ir_hid.sch.parts.txt
 *
 * Board (for protoboards):
 * \image html ir_hid.brd.png
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>



#include <string.h>

#include <stdlib.h>
#include <inttypes.h>
#include <stdint.h>

/* ----------------------- hardware I/O abstraction ------------------------ */

/* pin assignments:
 PB0  MOSI
 PB1  MISO
 PB2  SCK
 PB3
 PB4  XTAL1
 PB5  XTAL2
 PB6
 PB7  RESET_

 PA0  MIC_BIAS_IN
 PA1  MIC_IN
 PA2  RED_LED
 PA3
 PA4
 PA5
 PA6  STRIP_CLK
 PA7  STRIP_DO


 */


#define elements_of(array) (sizeof(array) / sizeof(array[0]))

#define LED_RED_CHANGE() PORTC ^= _BV(PC5)
#define LED_RED_ON() PORTC |= _BV(PC5)
#define LED_RED_OFF() PORTC &= ~_BV(PC5)
#define LED_RED_INIT() DDRC |= _BV(PC5);PORTB &= ~_BV(PC5)


#define STRIP_DATA_1() PORTD |= _BV(PD4)
#define STRIP_DATA_0() PORTD &= ~_BV(PD4)
#define STRIP_CLK_HIGH() PORTD |= _BV(PD3)
#define STRIP_CLK_LOW() PORTD &= ~_BV(PD3)

#define STRIP_CHANGE() PORTD ^= _BV(PD4) | _BV(PD3)

#define STRIP_INIT() DDRD |= _BV(PD4) | _BV(PD3);PORTD &= ~(_BV(PD4) | _BV(PD3))




/*
 * help macros
 */
#define STATIC_ASSERT(expr) extern char static_assert[ (!!(expr))*2 - 1]





typedef uint8_t rgb332_t;

/*
static inline
#define RGB888_RGB332(r,g,b)
*/


struct RGB888 {
	uint8_t red_;
	uint8_t green_;
	uint8_t blue_;
};

static uint8_t unpack_color_red(rgb332_t rgb) {
	uint8_t base = rgb >> 5;
	base &= ~0b111;
	uint8_t res = base << 5;
	res |= base << 2;
	res |= base >> 1;
	return res;
}


static uint8_t unpack_color_green(rgb332_t rgb) {
	uint8_t base = rgb >> 2;
	base &= ~0b111;
	uint8_t res = base << 5;
	res |= base << 2;
	res |= base >> 1;
	return res;
}

static uint8_t unpack_color_blue(rgb332_t rgb) {
	uint8_t base = rgb;
	base &= ~0b11;
	uint8_t res = base << 6;
	res |= base << 4;
	res |= base << 2;
	res |= base << 0;
	return res;
}
#define nop() __builtin_avr_nop()  //asm volatile(" nop \n\t")


void send_strip_byte(uint8_t byteval) {
	uint8_t temp = byteval;
	for (uint8_t i=0;i<8;++i) {
		if (temp & 0b10000000) {
			STRIP_DATA_1();
		} else {
			STRIP_DATA_0();
		}
		nop();
		STRIP_CLK_HIGH();
		nop();
		STRIP_CLK_LOW();
		temp <<=1;
	}
}



void send_color(uint8_t r,uint8_t g,uint8_t b) {
	const uint8_t start_byte = 0xFF;
	send_strip_byte(start_byte);
	send_strip_byte(r);
	send_strip_byte(g);
	send_strip_byte(b);
}


/* ------------------------------------------------------------------------- */

#define NUM_HARDWARE_KEYS    17

#define NUM_KEYS (NUM_HARDWARE_KEYS + NUM_IR_KEYS)


static void setColor(int numLedsOn, uint8_t r,uint8_t g,uint8_t b) {
	const int totalLeds = 200;
	send_strip_byte(0);
	send_strip_byte(0);
	send_strip_byte(0);
	send_strip_byte(0);
	send_strip_byte(0);

//	send_color(0,255,0);
//	send_color(0,0,255);
//	send_color(255,0,0);
//	send_color(0,255,255);

	for (int i=0;i<numLedsOn;i++) {
		send_color(b,g,r);
	}
	for (int i=0;i<totalLeds;i++) {
		send_color(0,0,0);
	}
	for (int i=0;i<totalLeds;i++) {
		send_strip_byte(0xFF);
	}
}

enum {
	ADC_CHANNEL_POT = 4

};

int main(void)
{
	//hardwareInit();
	//wdt_enable(WDTO_2S);

	LED_RED_INIT();
	STRIP_INIT();

	   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz

	   ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

	   // No MUX values needed to be changed to use ADC0

	   ADCSRA |= (1 << ADFR);  // Set ADC to Free-Running Mode


	   ADMUX |= ADC_CHANNEL_POT;

	   ADCSRA |= (1 << ADEN);  // Enable ADC
	   ADCSRA |= (1 << ADSC);  // Start A2D Conversions


	sei();

	/* main event loop */
	//printf_P(PSTR("Booted!"));
	uint8_t ledState = 0;
	int ledsOn = 0;
	for (;;)
	{
		//Watchdog
		wdt_reset();
		LED_RED_CHANGE();
		ledState = ~ledState;
	   // STRIP_CHANGE();

		int leds = ADCH / 2;

		if (ledState || 1) {
			setColor(leds,15,10,0);
			ledsOn++;

		} else {
			//setColor(40, 0,0,0);
			setColor(0,255,140,0);

		}

		if (ledsOn > 40) ledsOn = 0;


		_delay_ms(1);


	}
	return 0;
}

/* ------------------------------------------------------------------------- */

