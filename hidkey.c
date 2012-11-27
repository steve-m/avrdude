/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 *
 * HID Keyboard programming driver
 * http://wiki.steve-m.de/hidkey_gpio
 *
 * Copyright (C) 2012 Steve Markgraf <steve@steve-m.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
/* $Id$ */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <pthread.h>

#include "avrdude.h"
#include "avr.h"
#include "pindefs.h"
#include "pgm.h"
#include "bitbang.h"
#include "hidkey.h"

#undef DEBUG

//#define DEBUG
#ifdef DEBUG
#define DEBUGP(args...)	fprintf(stderr, fmt, ## args)
#else
#define DEBUGP(args...)
#endif

#if defined(HAVE_LIBUSB_1_0)
# if defined(HAVE_LIBUSB_1_0_LIBUSB_H)
#  include <libusb-1.0/libusb.h>
# endif
#else
# error "hidkey needs libusb-1.0!"
#endif

static struct libusb_device_handle *devh = NULL;
static int do_exit = 0;
static uint8_t input_state = 0;

/* delay until a 'keypress' has been detected by the keyboard in the
 * worst case, highly device dependent, YMMV */
#define INPUT_POLL_DELAY	40000
#define INPUT_INTERRUPT_DELAY	50000
//#define USE_INTERRUPT		/* slower in my case */

#define OUTPUT_DELAY		500

/* HID protocol constants */
#define HID_REPORT_GET		0x01
#define HID_REPORT_SET		0x09

#define HID_GET_IDLE		0x02
#define HID_SET_IDLE		0x0a
#define HID_SET_PROTOCOL	0x0b

#define HID_INPUT		0x0100
#define HID_OUTPUT		0x0200
#define INFINITE_IDLE		0x0000

static int hidkey_find_device(PROGRAMMER * pgm)
{
	if (!pgm->usbvid || !pgm->usbvid) {
		fprintf(stderr, "USB VID or PID of device unconfigured,"
				" check your config!\n");
		return -1;
	}

	devh = libusb_open_device_with_vid_pid(NULL, pgm->usbvid, pgm->usbpid);
	if (devh > 0) {
		DEBUGP("[hidkey] opened HID keyboard\n");

		if (libusb_kernel_driver_active(devh, 0) == 1) {
			DEBUGP("detaching kernel driver for iface 0\n");
			libusb_detach_kernel_driver(devh, 0);
		}

		if (libusb_kernel_driver_active(devh, 1) == 1) {
			DEBUGP("detaching kernel driver for iface 1\n");
			libusb_detach_kernel_driver(devh, 1);
		}

		return 0;
	}

	return -EIO;
}

static pthread_t poll_thread;
static pthread_cond_t exit_cond = PTHREAD_COND_INITIALIZER;

static void *poll_thread_main(void *arg)
{
	int r = 0;
	DEBUGP("poll thread running\n");

	while (!do_exit) {
		struct timeval tv = { 1, 0 };
		r = libusb_handle_events_timeout(NULL, &tv);
		if (r < 0) {
			pthread_cond_signal(&exit_cond);
			break;
		}
	}

	DEBUGP("poll thread shutting down\n");
	return NULL;
}

#define INTR_LENGTH		8
#define EP_INTR			0x81
static struct libusb_transfer *irq_transfer = NULL;
static uint8_t irqbuf[INTR_LENGTH];

static void LIBUSB_CALL irq_cb(struct libusb_transfer *transfer)
{
	input_state = transfer->buffer[0];

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		DEBUGP(stderr, "irq transfer status %d?\n", transfer->status);
		do_exit = 2;
		libusb_free_transfer(transfer);
		irq_transfer = NULL;
		return;
	}

	DEBUGP("IRQ callback %02x\n", transfer->buffer[0]);

	if (libusb_submit_transfer(irq_transfer) < 0) {
		fprintf(stderr, "error submitting interrupt transfer!\n");
		do_exit = 2;
	}
}

static int hidkey_alloc_transfers(void)
{
	irq_transfer = libusb_alloc_transfer(0);
	if (!irq_transfer)
		return -ENOMEM;

	libusb_fill_interrupt_transfer(irq_transfer, devh, EP_INTR, irqbuf,
		sizeof(irqbuf), irq_cb, NULL, 0);

	return 0;
}

static int hidkey_setpin(PROGRAMMER * pgm, int pin, int value)
{
	int r;
	static uint8_t data = 0x00;
	int invert = pin & PIN_INVERSE;

	if ((pin & ~invert) > 3)
		return -1;

	if (!!value ^ !!invert)
		data &= ~(1 << (pin-1));
	else
		data |= (1 << (pin-1));

	usleep(OUTPUT_DELAY);
	r = libusb_control_transfer(devh, 0x21, HID_REPORT_SET,
				    HID_OUTPUT, 0x00, &data, 1, 0);

	DEBUGP("%s: pin %i, val %i, ret: %i\n", __FUNCTION__, pin, value, r);

	return r;
}

static int hidkey_getpin(PROGRAMMER * pgm, int pin)
{
	int r, val;
	uint8_t data[8];
	uint8_t *modifier_keys;
	int invert = pin & PIN_INVERSE;

	/* TODO cache outputs and return them if needed */
	if ((pin & ~invert) < 4)
		return -1;

#ifdef USE_INTERRUPT
	usleep(INPUT_INTERRUPT_DELAY);
	modifier_keys = &input_state;
#else
	usleep(INPUT_POLL_DELAY);
	r = libusb_control_transfer(devh, 0x21 | 0x80, HID_REPORT_GET,
				    HID_INPUT, 0x00, data, 8, 0);

	modifier_keys = &data[0];
#endif

	val = (*modifier_keys & (1 << (pin-4))) ? 1 : 0;

	DEBUGP("%s: pin %i, val %i\n", __FUNCTION__, pin, val);

	return (invert) ? !val : val;
}

static int hidkey_highpulsepin(PROGRAMMER * pgm, int pin)
{
	hidkey_setpin(pgm, pin, 1);
	hidkey_setpin(pgm, pin, 0);

	return 0;
}

static void hidkey_display(PROGRAMMER *pgm, const char *p)
{
}

static void hidkey_enable(PROGRAMMER *pgm)
{
}

static void hidkey_disable(PROGRAMMER *pgm)
{
}

static void hidkey_powerup(PROGRAMMER *pgm)
{
}

static void hidkey_powerdown(PROGRAMMER *pgm)
{
}

static int hidkey_open(PROGRAMMER *pgm, char *port)
{
	int r;

	r = libusb_init(NULL);
	if (r < 0) {
		fprintf(stderr, "Failed to initialize libusb\n");
		return -1;
	}

	r = hidkey_find_device(pgm);
	if (r < 0) {
		fprintf(stderr, "Could not find/open device\n");
		return -1;
	}

	r = libusb_claim_interface(devh, 0);
	if (r < 0) {
		fprintf(stderr, "usb_claim_interface error %d\n", r);
		return -1;
	}

	r = pthread_create(&poll_thread, NULL, poll_thread_main, NULL);
	if (r)
		return -1;

	r = hidkey_alloc_transfers();
	if (r < 0)
		return -1;

	r = libusb_submit_transfer(irq_transfer);
#if 0
	r = libusb_control_transfer(devh, 0x21, HID_SET_IDLE,
				    INFINITE_IDLE, 0x00, NULL, 0, 0);
#endif
	bitbang_check_prerequisites(pgm);

	return 0;
}

static void hidkey_close(PROGRAMMER *pgm)
{
	int r;

	pgm->setpin(pgm, pgm->pinno[PIN_AVR_RESET], 1);

	pthread_cond_signal(&exit_cond);

	if (devh) {
		r = libusb_cancel_transfer(irq_transfer);
		libusb_release_interface(devh, 0);
		libusb_close(devh);
		libusb_exit(NULL);
	}

	pthread_join(poll_thread, NULL);

	return;
}

const char hidkey_desc[] = "HID keyboard GPIO driver";

void hidkey_initpgm(PROGRAMMER *pgm)
{
	strcpy(pgm->type, "hidkey");

	pgm->rdy_led		= bitbang_rdy_led;
	pgm->err_led		= bitbang_err_led;
	pgm->pgm_led		= bitbang_pgm_led;
	pgm->vfy_led		= bitbang_vfy_led;
	pgm->initialize		= bitbang_initialize;
	pgm->display		= hidkey_display;
	pgm->enable		= hidkey_enable;
	pgm->disable		= hidkey_disable;
	pgm->powerup		= hidkey_powerup;
	pgm->powerdown		= hidkey_powerdown;
	pgm->program_enable	= bitbang_program_enable;
	pgm->chip_erase		= bitbang_chip_erase;
	pgm->cmd		= bitbang_cmd;
	pgm->cmd_tpi		= bitbang_cmd_tpi;
	pgm->open		= hidkey_open;
	pgm->close		= hidkey_close;
	pgm->setpin		= hidkey_setpin;
	pgm->getpin		= hidkey_getpin;
	pgm->highpulsepin	= hidkey_highpulsepin;
	pgm->read_byte		= avr_read_byte_default;
	pgm->write_byte		= avr_write_byte_default;
}
