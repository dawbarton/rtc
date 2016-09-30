/*
** Copyright (c) 2014, 2016, David A.W. Barton
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
** aksim.c
**
** This unit implements an interface to an RLS AksIM rotary magnetic encoder in
** advanced SPI mode.
**
** Usage assumptions/notes:
**
** Use of global resources:
**   1) The McSPI0 module is used.
*/

/* ************************************************************************ */
/* * Includes, defines and global variables ******************************* */
/* ************************************************************************ */
#include "aksim.h"
#include "rtc_util.h"
#include "beaglebone.h"
#include "soc_AM335x.h"
#include "mcspi.h"
#include "mcspi_beaglebone.h"
#include "interrupt.h"
#include "gpio_setup.h"
#include "dmtimer.h"
#include "dmtimer_pwm.h"
#include "rtc_user.h"

#include "uartStdio.h"
#include "consoleUtils.h"

#define AKSIM_SPI_CHANNEL 0
#define AKSIM_BITS_PER_WORD 20
#define AKSIM_BYTES_PER_WORD 4
#define AKSIM_WORD_COUNT 2
#define MCSPI_IN_CLK 48000000
#define MCSPI_OUT_FREQ 3000000

volatile unsigned int aksimBuffer;
volatile unsigned int aksimBufferReady;
volatile unsigned int aksimStatus;
volatile unsigned int aksimCRCError = 0;
void (*aksimDataHandler)(void);

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
/* Wait for a certain number of counter ticks */
static void waitfor(unsigned int);
/* Interrupt handler */
void aksimMcSPIIsr(void);
/* CRC */
unsigned char crc8_4B(unsigned int);

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void waitfor(unsigned int duration)
{
    unsigned int t0 = TIME;
    while ((TIME - t0) < duration);
}

/* ************************************************************************ */
void aksimSetup(void)
{
    /* ******************************************************************* */
    /* Initialise the data handling routine */
    aksimDataHandler = NULL;

    /* ******************************************************************* */
    /* McSPI: Initialise clocks and do pin-muxing */

    /* Enable the clocks for McSPI0 module.*/
    McSPI0ModuleClkConfig();

    DEBUGPRINT("\t+ AksIM: enabled clocks for SPI0\r\n");

    /* Perform Pin-Muxing for SPI0 Instance: SPI is mode 0 of these pins */
    GPIOPinMuxSetup(CONTROL_CONF_SPI0_SCLK, PAD_FS_RXE_NA_PUPDD(0));
    GPIOPinMuxSetup(CONTROL_CONF_SPI0_D0,   PAD_FS_RXD_NA_PUPDD(0));
    GPIOPinMuxSetup(CONTROL_CONF_SPI0_D1,   PAD_FS_RXE_PU_PUPDE(0));
    GPIOPinMuxSetup(CONTROL_CONF_SPI0_CS0,  PAD_FS_RXD_NA_PUPDD(0));

    DEBUGPRINT("\t+ AksIM: done pin-muxing\r\n");

    /* ******************************************************************* */
    /* McSPI: Sort out interrupts */

    /* Register McSPIIsr interrupt handler */
	IntRegister(SYS_INT_SPI0INT, aksimMcSPIIsr);

	/* Set Interrupt Priority */
	IntPrioritySet(SYS_INT_SPI0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable system interrupt in AINTC */
	IntSystemEnable(SYS_INT_SPI0INT);

    DEBUGPRINT("\t+ AksIM: interrupts for McSPI registered and enabled\r\n");

    /* ******************************************************************* */
    /* McSPI: Set up McSPI */

    /* Reset the McSPI instance */
    McSPIReset(SOC_SPI_0_REGS);

    /* Enable chip select pin */
    McSPICSEnable(SOC_SPI_0_REGS);

    /* Enable master mode of operation */
    McSPIMasterModeEnable(SOC_SPI_0_REGS);

    /* Perform the necessary configuration for master mode */
    /* MCSPI_DATA_LINE_COMM_MODE_7 = D0 & D1 disabled for output, receive on D1 */
    McSPIMasterModeConfig(SOC_SPI_0_REGS, MCSPI_SINGLE_CH,
                          MCSPI_RX_ONLY_MODE, MCSPI_DATA_LINE_COMM_MODE_7,
                          AKSIM_SPI_CHANNEL);

    /* Set D1 to be an input at module level*/
    HWREG(SOC_SPI_0_REGS + MCSPI_SYST) |= (1 << 9);

    /* Configure the McSPI bus clock depending on clock mode */
    /* MCSPI_CLK_MODE_1: AKSIM loads data on rising clk edge, read it on the falling
       clock edge; the first bit is loaded when CS goes low */
    McSPIClkConfig(SOC_SPI_0_REGS, MCSPI_IN_CLK, MCSPI_OUT_FREQ, AKSIM_SPI_CHANNEL,
                   MCSPI_CLK_MODE_1);

    /* Configure the word length */
    McSPIWordLengthSet(SOC_SPI_0_REGS, MCSPI_WORD_LENGTH(AKSIM_BITS_PER_WORD), AKSIM_SPI_CHANNEL);

    /* Set polarity of SPIEN (CS) to low */
    McSPICSPolarityConfig(SOC_SPI_0_REGS, MCSPI_CS_POL_LOW, AKSIM_SPI_CHANNEL);

    /* Delay the clock start after CS goes low (by 8 clock cycles; 2us is needed = 6 cycles at 3MHz)*/
    /* This isn't used since we force the CS anyway... */
    /* McSPIInitDelayConfig(SOC_SPI_0_REGS, MCSPI_INITDLY_8); */

    /* Enable the receiver FIFO of McSPI peripheral */
    McSPIRxFIFOConfig(SOC_SPI_0_REGS, MCSPI_RX_FIFO_ENABLE, AKSIM_SPI_CHANNEL);

    /* Disable the transmitter FIFO of McSPI peripheral */
    McSPITxFIFOConfig(SOC_SPI_0_REGS, MCSPI_TX_FIFO_DISABLE, AKSIM_SPI_CHANNEL);

    /* Set the interrupt levels for the FIFO to avoid RX_FULL interrupts (zero isn't a valid input!) */
    /* Interrupt level must be a multiple of the (rounded up) word size */
    McSPIFIFOTrigLvlSet(SOC_SPI_0_REGS, AKSIM_BYTES_PER_WORD*AKSIM_WORD_COUNT, 1, MCSPI_RX_ONLY_MODE);

    DEBUGPRINT("\t+ AksIM: McSPI configured\r\n");
}

/* ************************************************************************ */
void aksimCaptureGet(void)
{
    /* Set the number of words to transfer */
    McSPIWordCountSet(SOC_SPI_0_REGS, AKSIM_WORD_COUNT);

    /* Enable the Tx and End-of-word-count interrupts of McSPI */
    McSPIIntEnable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(AKSIM_SPI_CHANNEL) | MCSPI_INT_RX_FULL(AKSIM_SPI_CHANNEL) | MCSPI_INT_EOWKE);

    /* SPIEN (CS) line is forced to low state */
    McSPICSAssert(SOC_SPI_0_REGS, AKSIM_SPI_CHANNEL);

    /* Wait for 2us = 48 cycles of the 24MHz clock */
    waitfor(48);

    /* Enable the McSPI channel for communication */
    McSPIChannelEnable(SOC_SPI_0_REGS, AKSIM_SPI_CHANNEL);
}

/* ************************************************************************ */
void aksimMcSPIIsr(void)
{
    unsigned int intCode;

    intCode = McSPIIntStatusGet(SOC_SPI_0_REGS);
    aksimBufferReady = 0;

    while(intCode) {

        /* Check for TX_EMPTY (even in RX_ONLY mode, we must put some data in when asked) */
        if (MCSPI_INT_TX_EMPTY(AKSIM_SPI_CHANNEL) == (intCode & MCSPI_INT_TX_EMPTY(AKSIM_SPI_CHANNEL))) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(AKSIM_SPI_CHANNEL));

            /* Transmit some dummy data */
            McSPITransmitData(SOC_SPI_0_REGS, 0, AKSIM_SPI_CHANNEL);
        }

        if (MCSPI_INT_RX_FULL(AKSIM_SPI_CHANNEL) == (intCode & MCSPI_INT_RX_FULL(AKSIM_SPI_CHANNEL))) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_RX_FULL(AKSIM_SPI_CHANNEL));

            /* Extract the data from the FIFO */
            aksimBuffer = McSPIReceiveData(SOC_SPI_0_REGS, AKSIM_SPI_CHANNEL) & AKSIM_BIT_MASK;
            aksimStatus = McSPIReceiveData(SOC_SPI_0_REGS, AKSIM_SPI_CHANNEL) & AKSIM_BIT_MASK;
            aksimCRCError |= (aksimStatus & 0xFF) != crc8_4B((aksimBuffer << 12) | (aksimStatus >> 8));

            /* Disable the McSPI channel */
            McSPIChannelDisable(SOC_SPI_0_REGS, AKSIM_SPI_CHANNEL);

            /* Wait for 1us = 24 cycles of the 24 MHz clock */
            waitfor(24);

            /* Force SPIEN (CS) line to the inactive state */
            McSPICSDeAssert(SOC_SPI_0_REGS, AKSIM_SPI_CHANNEL);

            /* Disable interrupts */
            McSPIIntDisable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(AKSIM_SPI_CHANNEL) | MCSPI_INT_RX_FULL(AKSIM_SPI_CHANNEL) | MCSPI_INT_EOWKE);

            /* Signal data ready */
            aksimBufferReady = 1;

        }

        /* Check for end-of-word-count */
        if (MCSPI_INT_EOWKE == (intCode & MCSPI_INT_EOWKE)) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_EOWKE);

        }

        intCode = McSPIIntStatusGet(SOC_SPI_0_REGS);
    }

    /* Call the data handler if necessary */
    if (aksimBufferReady && (aksimDataHandler != NULL))
        aksimDataHandler();

}

/* ************************************************************************ */
unsigned char crc8_4B(unsigned int bb)
{
    // use this function to calculate CRC from 32-bit number
    static unsigned char tableCRC [256] = {
        0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
        0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
        0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
        0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
        0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
        0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
        0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
        0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
        0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
        0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
        0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
        0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
        0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
        0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
        0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
        0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C};
    unsigned char crc;
    unsigned int t;
    t = (bb >> 24) & 0x000000FF;
    crc = ((bb >> 16) & 0x000000FF);
    t = crc ^ tableCRC[t];
    crc = ((bb >> 8) & 0x000000FF);
    t = crc ^ tableCRC[t];
    crc = (bb & 0x000000FF);
    t = crc ^ tableCRC[t];
    crc = tableCRC[t];
    return crc;
}
