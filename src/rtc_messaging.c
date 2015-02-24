/*
** Copyright (c) 2014, David A.W. Barton
** (david.barton@bristol.ac.uk) All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**
** 1. Redistributions of source code must retain the above copyright
** notice, this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright
** notice, this list of conditions and the following disclaimer in the
** documentation and/or other materials provided with the distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
  USB Message format
  ==================
  command                     2 bytes
  length of command payload   4 bytes
  command payload             variable (may be zero)
  checksum                    4 bytes (if command payload is not zero)
*/

/* ************************************************************************ */
/* * Includes, defines and global variables ******************************* */
/* ************************************************************************ */
#include "rtc_messaging.h"
#include "rtc_data.h"
#include "usb_setup.h"
#include "usb_bulk_structs.h"
#include "rtc_util.h"

#define CMD_GET_PAR_NAMES     10
#define CMD_GET_PAR_SIZES     11
#define CMD_GET_PAR_TYPES     12
#define CMD_GET_PAR_VALUE     20
#define CMD_SET_PAR_VALUE     25
#define CMD_GET_STREAM        30
#define CMD_SOFT_RESET        90

#define USB_RX_BUFFER g_sRxBulkBuffer
#define USB_TX_BUFFER g_sTxBulkBuffer

#define RX_TIMEOUT 24000000  /* 1 second with 24 MHz timer */
#define TX_TIMEOUT 24000000  /* 1 second with 24 MHz timer */
#define ERROR_TIMEOUT 12000000  /* 0.5 seconds with 24 MHz timer */

#define CRC32_INITIAL 0xFFFFFFFF

/* RX buffers */
unsigned int rx_size;
unsigned short rx_cmd;
unsigned char rx_buffer[MAX_RX_BUFFER];
unsigned int rx_crc;

/* TX buffers */
unsigned int tx_size;
unsigned char tx_buffer[MAX_TX_BUFFER];
void *tx_buffer_ptr;
unsigned int tx_crc;

/* CRC table */
static unsigned int crc_table[256];


/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
static int process_message(void);
static void rx_error(void);
static int get_data(unsigned int len, void *buffer, unsigned int *idx);
static int put_data(unsigned int len, void *buffer, unsigned int *idx);
static void crc32_setup(void);
static void crc32_update(unsigned int *crc, unsigned int len, unsigned char *data);
static void crc32_final(unsigned int *crc);

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void rtc_handle_messages(void)
{
	/* USB ring buffer information */
	tUSBRingBufObject sRxRing, sTxRing;
	unsigned int idxRx, idxTx;

	/* Temporary value for the CRC */
	unsigned int rx_crc_check;

	/* Get the necessary information */
	USBBufferInfoGet(&USB_RX_BUFFER, &sRxRing);
	idxRx = sRxRing.ulReadIndex;
	USBBufferInfoGet(&USB_TX_BUFFER, &sTxRing);
	idxTx = sTxRing.ulWriteIndex;

	/* Set up CRC tables */
	crc32_setup();

	while (1) {
		if (!USBConfigured)
			continue;
		/* Get number of bytes received */
		if (USBBufferDataAvailable(&USB_RX_BUFFER) == 0)
			continue;
		/* Get the command */
		if (get_data(2, &rx_cmd, &idxRx) != 0) { rx_error(); continue; }
		/* Get the payload size */
		if (get_data(4, &rx_size, &idxRx) != 0) { rx_error(); continue; }
		/* Check that the length is sane */
		if (rx_size > MAX_RX_BUFFER) { rx_error(); continue; }
		/* Get the command body if necessary and crc check it */
		if (rx_size > 0) {
			if (get_data(rx_size, rx_buffer, &idxRx) != 0) { rx_error(); continue; }
			if (get_data(4, &rx_crc, &idxRx) != 0) { rx_error(); continue; }
			/* Calculate the checksum on the data */
			rx_crc_check = CRC32_INITIAL;
			crc32_update(&rx_crc_check, rx_size, rx_buffer);
			crc32_final(&rx_crc_check);
			/* Check the checksum */
			if (rx_crc != rx_crc_check) { rx_error(); continue; }
		}
		/* Process the message */
		tx_size = 0;
		if (process_message() == 0) {
			/* Send any response needed */
			if (tx_size > 0) {
				/* Send a message with a payload */
				tx_crc = CRC32_INITIAL;
				crc32_update(&tx_crc, tx_size, tx_buffer_ptr);
				crc32_final(&tx_crc);
				if (put_data(4, &tx_size, &idxTx) != 0) continue;
				if (put_data(tx_size, tx_buffer_ptr, &idxTx) != 0) continue;
				if (put_data(4, &tx_crc, &idxTx) != 0) continue;
			}
			else {
				/* Send a simple acknowledgment */
				tx_buffer[0] = 0xFF;
				tx_buffer[1] = 0xAB;
				tx_buffer[2] = 0xCD;
				tx_buffer[3] = 0xFF;
				put_data(4, tx_buffer, &idxTx);
			}
		}
	}
}

/* ************************************************************************ */
int process_message(void)
{
	switch (rx_cmd) {
		case CMD_GET_PAR_NAMES:
			return rtc_data_get_par_names();
			break;
		case CMD_GET_PAR_SIZES:
			return rtc_data_get_par_sizes();
			break;
		case CMD_GET_PAR_TYPES:
			return rtc_data_get_par_types();
			break;
		case CMD_GET_PAR_VALUE:
			return rtc_data_get_par_value();
			break;
		case CMD_SET_PAR_VALUE:
			return rtc_data_set_par_value();
			break;
		case CMD_GET_STREAM:
			return rtc_data_get_stream();
			break;
		case CMD_SOFT_RESET:
			break;
	}
	return -1;
}


/* ************************************************************************ */
void rx_error(void)
{
	unsigned int lastTime, receivedBytes;
	lastTime = TIME;
	do {
		/* Discard any data received in this time */
		receivedBytes = USBBufferDataAvailable(&USB_RX_BUFFER);
		if (receivedBytes > 0)
			USBBufferDataRemoved(&USB_RX_BUFFER, receivedBytes);
	} while (TIME - lastTime < ERROR_TIMEOUT);
}

/* ************************************************************************ */
int get_data(unsigned int len, void *buffer, unsigned int *idx)
{
	unsigned int receivedBytes, getBytes, lastTime = TIME;
	unsigned char *ptr = (unsigned char *)buffer;
	do {
		receivedBytes = USBBufferDataAvailable(&USB_RX_BUFFER);
		if (receivedBytes > 0) {
			lastTime = TIME;
			if (receivedBytes > len) {
				getBytes = len;
				receivedBytes = len;
			}
			else
				getBytes = receivedBytes;
			while (getBytes--) {
				*ptr++ = g_pucUSBRxBulkBuffer[(*idx)++];
				*idx = (*idx == USB_RX_BULK_BUFFER_SIZE) ? 0 : *idx;
			}
			USBBufferDataRemoved(&USB_RX_BUFFER, receivedBytes);
			len -= receivedBytes;
		}
	} while ((len > 0) && (TIME - lastTime <= RX_TIMEOUT));
	if (len > 0)
		return -1;
	else
		return 0;
}

/* ************************************************************************ */
int put_data(unsigned int len, void *buffer, unsigned int *idx)
{
	unsigned int availableSpace, putBytes, lastTime = TIME;
	unsigned char *ptr = (unsigned char *)buffer;
	do {
		availableSpace = USBBufferSpaceAvailable(&USB_TX_BUFFER);
		if (availableSpace > 0) {
			lastTime = TIME;
			if (availableSpace > len) {
				putBytes = len;
				availableSpace = len;
			}
			else
				putBytes = availableSpace;
			while (putBytes--) {
				g_pucUSBTxBulkBuffer[(*idx)++] = *ptr++;
				*idx = (*idx == USB_TX_BULK_BUFFER_SIZE) ? 0 : *idx;
			}
			USBBufferDataWritten(&USB_TX_BUFFER, availableSpace);
			len -= availableSpace;
		}
	} while ((len > 0) && (TIME - lastTime <= RX_TIMEOUT));
	if (len > 0)
		return -1;
	else
		return 0;
}

/* ************************************************************************ */
void crc32_setup(void)
{
	/* This CRC implementation is the same as is used for PNG images */
	unsigned int c;
	unsigned int n, k;

	for (n = 0; n < 256; n++) {
		c = n;
		for (k = 0; k < 8; k++) {
			if (c & 1)
				c = 0xEDB88320L ^ (c >> 1);
			else
				c = c >> 1;
		}
		crc_table[n] = c;
	}
}

/* ************************************************************************ */
void crc32_update(unsigned int *crc, unsigned int len, unsigned char *data)
{
	unsigned int i;
	for (i = 0; i < len; i++) {
		*crc = crc_table[(*crc ^ data[i]) & 0xFF] ^ (*crc >> 8);
	}
}

/* ************************************************************************ */
void crc32_final(unsigned int *crc)
{
	*crc ^= 0xFFFFFFFF;
}
