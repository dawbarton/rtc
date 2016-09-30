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
void (*aksimDataHandler)(void);

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
/* Wait for a certain number of counter ticks */
static void waitfor(unsigned int);
/* Interrupt handler */
void aksimMcSPIIsr(void);

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
