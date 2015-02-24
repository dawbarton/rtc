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
#include "usb_setup.h"

#include "interrupt.h"
#include "delay.h"
#include "usb_bulk_structs.h"
#include "consoleUtils.h"   

volatile int USBConfigured;
volatile int USBDataAvail;

static unsigned char g_pucDescriptorData[DESCRIPTOR_DATA_SIZE];

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */

/* Initialise the USB bulk device */
void usb_init()
{
	/* Zero status variables */
	USBConfigured = 0;
	USBDataAvail = 0;

	/* USB module clock enable */
    USB0ModuleClkConfig();

    /* Registering the Interrupt Service Routine (ISR) */
    IntRegister(SYS_INT_USB0, USB0DeviceIntHandler);

    /* Setting the priority for the system interrupt in AINTC - set it high to allow USB to be prempted */
    IntPrioritySet(SYS_INT_USB0, 127, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enabling the system interrupt in AINTC */
    IntSystemEnable(SYS_INT_USB0);

    /* Enable the delay timer */
    DelayTimerSetup();

    /* Initialize the transmit and receive buffers */
    USBBufferInit((tUSBBuffer *)&g_sTxBulkBuffer);
    USBBufferInit((tUSBBuffer *)&g_sRxBulkBuffer);
    USBBufferInit((tUSBBuffer *)&g_sTxCDCBuffer);
    USBBufferInit((tUSBBuffer *)&g_sRxCDCBuffer);

    /* Register the two instances */
    g_psCompDevices[0].pvInstance =
        USBDBulkCompositeInit(0, (tUSBDBulkDevice *)&g_sBulkDevice);
    g_psCompDevices[1].pvInstance =
        USBDCDCCompositeInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);

    /* Register our device and place it on the bus */
    USBDCompositeInit(0, &g_sCompDevice, DESCRIPTOR_DATA_SIZE,
                      g_pucDescriptorData);
}

/* ************************************************************************ */
/* TX handler for USB bulk device */
unsigned int USBBulkTxHandler(void *pvCBData, unsigned int ulEvent, 
		unsigned int ulMsgValue, void *pvMsgData)
{
	/* We could handle the USB_EVENT_TX_COMPLETE event but there is no need */
	return 0;
}

/* ************************************************************************ */
/* RX handler for USB bulk device */
unsigned int USBBulkRxHandler(void *pvCBData, unsigned int ulEvent,
    	unsigned int ulMsgValue, void *pvMsgData)
{
    switch (ulEvent)
    {
        case USB_EVENT_CONNECTED:
        	/* Now connected so flush the stale buffers */
            USBBufferFlush(&g_sTxBulkBuffer);
            USBBufferFlush(&g_sRxBulkBuffer);
            USBConfigured = 1;
            break;
        case USB_EVENT_DISCONNECTED:
        	/* Now disconnected - since the device is bus powered this shouldn't happen except for a reset */
            USBConfigured = 0;
            break;
        case USB_EVENT_RX_AVAILABLE:
        	/* Data available - read it at the next available opportunity */
        	USBDataAvail = 1;
        	break;
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        	// ignore suspend and resume
            break;
        default:
        	// ignore all other events
            break;
    }
    return 0;
}

/* ************************************************************************ */
/* Control handler for CDC device */
unsigned int USBCDCControlHandler(void *pvCBData, unsigned int ulEvent,
                            unsigned int ulMsgValue, void *pvMsgData)
{
    /* Which event are we being asked to process? */
    switch(ulEvent)
    {
        /* We are connected to a host and communication is now possible. */
        case USB_EVENT_CONNECTED:
            /* Flush our buffers. */
            USBBufferFlush(&g_sRxCDCBuffer);
            USBBufferFlush(&g_sTxCDCBuffer);
            break;
        /* The host has disconnected. */
        case USB_EVENT_DISCONNECTED:
            break;
        default:
            break;
    }
    return 0;
}

/* ************************************************************************ */
/* TX handler for USB CDC device */
unsigned int USBCDCTxHandler(void *pvCBData, unsigned int ulEvent, 
        unsigned int ulMsgValue, void *pvMsgData)
{
    /* We could handle the USB_EVENT_TX_COMPLETE event but there is no need */
    return 0;
}

/* ************************************************************************ */
/* RX handler for USB CDC device */
unsigned int USBCDCRxHandler(void *pvCBData, unsigned int ulEvent,
        unsigned int ulMsgValue, void *pvMsgData)
{
    switch (ulEvent)
    {
        case USB_EVENT_RX_AVAILABLE:
            break;
        case USB_EVENT_DATA_REMAINING:
            break;
        default:
            // ignore all other events
            break;
    }
    return 0;
}

/* ************************************************************************ */
/* Event handler for the composite device */
unsigned int USBCompEventHandler(void *pvCBData, unsigned int ulEvent,
                              unsigned int ulMsgValue, void *pvMsgData)
{
    /*  Which event are we being asked to process?   */
    switch(ulEvent)
    {
        /* We are connected to a host and communication is now possible. */
        case USB_EVENT_CONNECTED:
            break;
        /* The host has disconnected. */
        case USB_EVENT_DISCONNECTED:
            break;
        default:
            break;
    }
    return 0 ;
}
