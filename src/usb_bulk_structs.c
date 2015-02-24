//*****************************************************************************
//
// usb_bulk_structs.c - Data structures defining this bulk USB device.
//
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

#include "usb_bulk_structs.h"
#include "usb_setup.h"

//*****************************************************************************
//
// The languages supported by this device.
//
//*****************************************************************************
const unsigned char g_pLangDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

//*****************************************************************************
//
// The manufacturer string.
//
//*****************************************************************************
const unsigned char g_pManufacturerString[] =
{
    (21 + 1) * 2,
    USB_DTYPE_STRING,
    'U', 0, 'n', 0, 'i', 0, 'v', 0, 'e', 0, 'r', 0, 's', 0, 'i', 0, 't', 0, 'y', 0, 
    ' ', 0, 'o', 0, 'f', 0, ' ', 0, 'B', 0, 'r', 0, 'i', 0, 's', 0, 't', 0, 'o', 0, 
    'l', 0
};

//*****************************************************************************
//
// The product string.
//
//*****************************************************************************
const unsigned char g_pProductString[] =
{
    (20 + 1) * 2,
    USB_DTYPE_STRING,
    'R', 0, 'e', 0, 'a', 0, 'l', 0, '-', 0, 't', 0, 'i', 0, 'm', 0, 'e', 0, ' ', 0, 
    'c', 0, 'o', 0, 'n', 0, 't', 0, 'r', 0, 'o', 0, 'l', 0, 'l', 0, 'e', 0, 'r', 0
};

//*****************************************************************************
//
// The serial number string.
//
//*****************************************************************************
const unsigned char g_pSerialNumberString[] =
{
    (8 + 1) * 2,
    USB_DTYPE_STRING,
    '1', 0, '6', 0, '7', 0, '0', 0, '7', 0, '2', 0, '2', 0, '0', 0
};

//*****************************************************************************
//
// The data interface description string.
//
//*****************************************************************************
const unsigned char g_pDataInterfaceString[] =
{
    (17 + 1) * 2,
    USB_DTYPE_STRING,
    'C', 0, 'o', 0, 'n', 0, 't', 0, 'r', 0, 'o', 0, 'l', 0, ' ', 0, 'i', 0, 'n', 0, 
    't', 0, 'e', 0, 'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0
};

//*****************************************************************************
//
// The configuration description string.
//
//*****************************************************************************
const unsigned char g_pConfigString[] =
{
    (21 + 1) * 2,
    USB_DTYPE_STRING,
    'C', 0, 'o', 0, 'n', 0, 't', 0, 'r', 0, 'o', 0, 'l', 0, ' ', 0, 'c', 0, 'o', 0, 
    'n', 0, 'f', 0, 'i', 0, 'g', 0, 'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0, 'o', 0, 
    'n', 0
};

//*****************************************************************************
//
// The descriptor string table.
//
//*****************************************************************************
const unsigned char * const g_pStringDescriptors[] =
{
    g_pLangDescriptor,
    g_pManufacturerString,
    g_pProductString,
    g_pSerialNumberString,
    g_pDataInterfaceString,
    g_pConfigString
};

#define NUM_STRING_DESCRIPTORS (sizeof(g_pStringDescriptors) /                \
                                sizeof(unsigned char *))

//*****************************************************************************
//
// The bulk device initialization and customization structures. In this case,
// we are using USBBuffers between the bulk device class driver and the
// application code. The function pointers and callback data values are set
// to insert a buffer in each of the data channels, transmit and receive.
//
// With the buffer in place, the bulk channel callback is set to the relevant
// channel function and the callback data is set to point to the channel
// instance data. The buffer, in turn, has its callback set to the application
// function and the callback data set to our bulk instance structure.
//
//*****************************************************************************
tBulkInstance g_sBulkInstance;

extern const tUSBBuffer g_sTxBulkBuffer;
extern const tUSBBuffer g_sRxBulkBuffer;

const tUSBDBulkDevice g_sBulkDevice =
{
    0x0123,  // vendor ID
    0x4567,  // product ID
    0,       // milli-amps drawn from the host
    USB_CONF_ATTR_SELF_PWR,  // self powered
    USBBufferEventCallback,
    (void *)&g_sRxBulkBuffer,
    USBBufferEventCallback,
    (void *)&g_sTxBulkBuffer,
    g_pStringDescriptors,
    NUM_STRING_DESCRIPTORS,
    &g_sBulkInstance
};

//*****************************************************************************
//
// Receive buffer (from the USB perspective).
//
//*****************************************************************************
unsigned char g_pucUSBRxBulkBuffer[USB_RX_BULK_BUFFER_SIZE];
unsigned char g_pucRxBulkBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
const tUSBBuffer g_sRxBulkBuffer =
{
    false,                           // This is a receive buffer.
    USBBulkRxHandler,                // pfnCallback
    (void *)&g_sBulkDevice,          // Callback data is our device pointer.
    USBDBulkPacketRead,              // pfnTransfer
    USBDBulkRxPacketAvailable,       // pfnAvailable
    (void *)&g_sBulkDevice,          // pvHandle
    g_pucUSBRxBulkBuffer,            // pcBuffer
    USB_RX_BULK_BUFFER_SIZE,         // ulBufferSize
    g_pucRxBulkBufferWorkspace       // pvWorkspace
};

//*****************************************************************************
//
// Transmit buffer (from the USB perspective).
//
//*****************************************************************************
unsigned char g_pucUSBTxBulkBuffer[USB_TX_BULK_BUFFER_SIZE];
unsigned char g_pucTxBulkBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
const tUSBBuffer g_sTxBulkBuffer =
{
    true,                            // This is a transmit buffer.
    USBBulkTxHandler,                // pfnCallback
    (void *)&g_sBulkDevice,          // Callback data is our device pointer.
    USBDBulkPacketWrite,             // pfnTransfer
    USBDBulkTxPacketAvailable,       // pfnAvailable
    (void *)&g_sBulkDevice,          // pvHandle
    g_pucUSBTxBulkBuffer,            // pcBuffer
    USB_TX_BULK_BUFFER_SIZE,         // ulBufferSize
    g_pucTxBulkBufferWorkspace       // pvWorkspace
};


/****************************************************************************
*
* The CDC device initialization and customization structures. In this case,
* we are using USBBuffers between the CDC device class driver and the
* application code. The function pointers and callback data values are set
* to insert a buffer in each of the data channels, transmit and receive.
* With the buffer in place, the CDC channel callback is set to the relevant
* channel function and the callback data is set to point to the channel
* instance data. The buffer, in turn, has its callback set to the application
* function and the callback data set to our CDC instance structure.
*
****************************************************************************/
tCDCSerInstance g_sCDCInstance;

const tUSBDCDCDevice g_sCDCDevice =
{
    0x0123,  // vendor ID
    0x4567,  // product ID
    0,       // milli-amps drawn from the host
    USB_CONF_ATTR_SELF_PWR,  // self powered
    USBCDCControlHandler,
    (void *)&g_sCDCDevice,
    USBBufferEventCallback,
    (void *)&g_sRxCDCBuffer,
    USBBufferEventCallback,
    (void *)&g_sTxCDCBuffer,
    0,
    0,
    &g_sCDCInstance
};

/****************************************************************************
*
* Receive buffers.
*
****************************************************************************/
unsigned char g_pucUSBRxCDCBuffer[USB_RX_CDC_BUFFER_SIZE];
unsigned char g_pucRxCDCBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
const tUSBBuffer g_sRxCDCBuffer =
{
    false,                          // This is a receive buffer.
    USBCDCRxHandler,                // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketRead,              // pfnTransfer
    USBDCDCRxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pucUSBRxCDCBuffer,            // pcBuffer
    USB_RX_CDC_BUFFER_SIZE,         // ulBufferSize
    g_pucRxCDCBufferWorkspace       // pvWorkspace
};

/****************************************************************************
*
* Transmit buffers.
*
****************************************************************************/
unsigned char g_pucUSBTxCDCBuffer[USB_TX_CDC_BUFFER_SIZE];
unsigned char g_pucTxCDCBufferWorkspace[USB_BUFFER_WORKSPACE_SIZE];
const tUSBBuffer g_sTxCDCBuffer =
{
    true,                           // This is a transmit buffer.
    USBCDCTxHandler,                // pfnCallback
    (void *)&g_sCDCDevice,          // Callback data is our device pointer.
    USBDCDCPacketWrite,             // pfnTransfer
    USBDCDCTxPacketAvailable,       // pfnAvailable
    (void *)&g_sCDCDevice,          // pvHandle
    g_pucUSBTxCDCBuffer,            // pcBuffer
    USB_TX_CDC_BUFFER_SIZE,         // ulBufferSize
    g_pucTxCDCBufferWorkspace       // pvWorkspace
};


/****************************************************************************
*
* The array of devices supported by this composite device.
*
****************************************************************************/
tCompositeEntry g_psCompDevices[NUM_DEVICES] =
{

    /* Bulk Device Instance. */

    {
        &g_sBulkDeviceInfo,
        0
    },

    /* Serial Device Instance. */

    {
        &g_sCDCSerDeviceInfo,
        0
    }
};

/****************************************************************************
*
* Additional workspaced required by the composite device.
*
****************************************************************************/
unsigned int g_pulCompWorkspace[NUM_DEVICES];

/****************************************************************************
*
* The instance data for this composite device.
*
****************************************************************************/
tCompositeInstance g_CompInstance;

/****************************************************************************
*
* Allocate the Device Data for the top level composite device class.
*
****************************************************************************/
tUSBDCompositeDevice g_sCompDevice =
{
    0x0123,  // vendor ID
    0x4567,  // product ID
    500,
    USB_CONF_ATTR_BUS_PWR,
    USBCompEventHandler,
    g_pStringDescriptors,
    NUM_STRING_DESCRIPTORS,
    NUM_DEVICES,
    g_psCompDevices,
    &g_CompInstance
};
