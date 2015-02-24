//*****************************************************************************
//
// usb_bulk_structs.h - Data structures defining this bulk USB device.
//
// This is part of revision 6288 of the DK-LM3S9B96 Firmware Package.
//
//*****************************************************************************

// Modifications for BBB-DAQ by David AW Barton (2014)

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


#ifndef _USB_BULK_STRUCTS_H_
#define _USB_BULK_STRUCTS_H_

#include "hw_types.h"
#include "usblib.h"
#include "usb.h"
#include "usbdevice.h"
#include "usbdevice.h"
#include "usbcdc.h"
#include "usbdcdc.h"
#include "usbdbulk.h"
#include "usbdcomp.h"
#include "hw_usb.h"

//*****************************************************************************
//
// The size of the transmit and receive buffers used. The number is chosen pretty
// much at random though the buffer should be at least twice the size of
// a maximum-sized USB packet.
//
//*****************************************************************************
#define USB_RX_BULK_BUFFER_SIZE 1024
#define USB_TX_BULK_BUFFER_SIZE 1024

extern const tUSBBuffer g_sTxBulkBuffer;
extern const tUSBBuffer g_sRxBulkBuffer;
extern const tUSBDBulkDevice g_sBulkDevice;
extern unsigned char g_pucUSBTxBulkBuffer[];
extern unsigned char g_pucUSBRxBulkBuffer[];

#define USB_RX_CDC_BUFFER_SIZE 1024
#define USB_TX_CDC_BUFFER_SIZE 1024

extern const tUSBBuffer g_sTxCDCBuffer;
extern const tUSBBuffer g_sRxCDCBuffer;
extern const tUSBDCDCDevice g_sCDCDevice;
extern unsigned char g_pucUSBTxCDCBuffer[];
extern unsigned char g_pucUSBRxCDCBuffer[];

#define   NUM_DEVICES            2

extern tCompositeEntry g_psCompDevices[NUM_DEVICES];
extern tUSBDCompositeDevice g_sCompDevice;

#endif
