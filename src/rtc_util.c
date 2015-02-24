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

/* ************************************************************************ */
/* * Includes, defines and global variables ******************************* */
/* ************************************************************************ */
#include "rtc_util.h"
#include <string.h>
#include "usb_setup.h"
#include "usb_bulk_structs.h"
#include "ustdlib.h"
#include "gpio_setup.h"

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void usb_print(const char *text)
{
    /* USB ring buffer information */
    tUSBRingBufObject sTxRing;
    unsigned int idxBuffer, available;

    /* String to send */
    unsigned int idxStr, len = strlen(text);

    /* Get the ring buffer and index */
	USBBufferInfoGet(&g_sTxCDCBuffer, &sTxRing);
	idxBuffer = sTxRing.ulWriteIndex;

	/* See how much space is available */
	available = USBBufferSpaceAvailable(&g_sTxCDCBuffer);
	if (available >= len) {
		available = len;
		idxStr = 0;
		while (len--) {
			g_pucUSBTxCDCBuffer[idxBuffer++] = text[idxStr++];
			idxBuffer = (idxBuffer == USB_TX_CDC_BUFFER_SIZE) ? 0 : idxBuffer;
		}
		USBBufferDataWritten(&g_sTxCDCBuffer, available);
	}	
}

/* ************************************************************************ */
void usb_printf(const char *text, ...)
{
	char pcBuf[1024];
    va_list vaArgP;
    va_start(vaArgP, text);
    uvsnprintf(pcBuf, 1024, text, vaArgP);
    va_end(vaArgP);
    usb_print(pcBuf);
}


/* ************************************************************************ */
void rtc_led(unsigned int led, unsigned int state)
{
    if (led < 4)
        GPIOPinWrite(SOC_GPIO_1_REGS, 21 + led, state);
}
