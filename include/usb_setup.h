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
#ifndef __USB_SETUP_H__
#define __USB_SETUP_H__

#include "hw_types.h"
#include "usblib.h"
#include "usb.h"
#include "usbdevice.h"
#include "usbdevice.h"
#include "usbcdc.h"
#include "usbdcdc.h"
#include "usbdbulk.h"

extern volatile int USBConfigured;
extern volatile int USBDataAvail;

#define DESCRIPTOR_DATA_SIZE    (COMPOSITE_DBULK_SIZE + COMPOSITE_DCDC_SIZE)

void usb_init();

unsigned int USBBulkTxHandler(void *pvCBData, unsigned int ulEvent, 
		unsigned int ulMsgValue, void *pvMsgData);
unsigned int USBBulkRxHandler(void *pvCBData, unsigned int ulEvent,
    	unsigned int ulMsgValue, void *pvMsgData);

unsigned int USBCDCControlHandler(void *pvCBData, unsigned int ulEvent,
        unsigned int ulMsgValue, void *pvMsgData);
unsigned int USBCDCTxHandler(void *pvCBData, unsigned int ulEvent, 
		unsigned int ulMsgValue, void *pvMsgData);
unsigned int USBCDCRxHandler(void *pvCBData, unsigned int ulEvent,
    	unsigned int ulMsgValue, void *pvMsgData);

unsigned int USBCompEventHandler(void *pvCBData, unsigned int ulEvent,
        unsigned int ulMsgValue, void *pvMsgData);

#endif