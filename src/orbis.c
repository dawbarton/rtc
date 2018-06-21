/*
** Copyright (c) 2018, David A.W. Barton
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
#include "orbis.h"
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

#include "ad760x.h"

#define ORBIS_SPI_CHANNEL 1
#define MCSPI_IN_CLK 48000000
#define MCSPI_OUT_FREQ 3000000

#define CS_PIN_MUX GPIO_1_19
#define CS_REGS SOC_GPIO_1_REGS
#define CS_PIN 19

volatile unsigned char orbisBuffer[ORBIS_WORD_COUNT];
volatile int orbisBufferReady = 0;

volatile unsigned int orbisMultiturn;
volatile unsigned int orbisPosition;
volatile float orbisAngle;
volatile unsigned int orbisStatus;
volatile unsigned int orbisCRC;


/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
/* Wait for a certain number of counter ticks */
static void waitfor(unsigned int);
/* Get conversion data from the device */
static void orbisMcSPIIsr(void);

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void waitfor(unsigned int duration)
{
    unsigned int t0 = TIME;
    while ((TIME - t0) < duration);
}

/* ************************************************************************ */
void orbisSetup(void)
{
    /* This assumes that the basic set up has been done by the ad5064 code */

    /* Perform Pin-Muxing for CS - use GPIO */
    GPIOPinMuxSetup(CS_PIN_MUX, PAD_FS_RXD_NA_PUPDD(7));
    GPIODirModeSet(CS_REGS, CS_PIN, GPIO_DIR_OUTPUT);
    GPIOPinWrite(CS_REGS, CS_PIN, GPIO_PIN_HIGH);
    DEBUGPRINT("\t+ Orbis: done pin-muxing\r\n");

    /* Register McSPIIsr interrupt handler */
    IntRegister(SYS_INT_SPI0INT, orbisMcSPIIsr);
    /* Set Interrupt Priority */
    IntPrioritySet(SYS_INT_SPI0INT, 0, AINTC_HOSTINT_ROUTE_IRQ);
    /* Enable system interrupt in AINTC */
    IntSystemEnable(SYS_INT_SPI0INT);
    DEBUGPRINT("\t+ Orbis: interrupts for McSPI registered and enabled\r\n");

    /* Perform the necessary configuration for master mode */
    /* MCSPI_DATA_LINE_COMM_MODE_7 = D0 & D1 disabled for output, receive on D1 */
    McSPIMasterModeConfig(SOC_SPI_0_REGS, MCSPI_MULTI_CH,
                          MCSPI_RX_ONLY_MODE, MCSPI_DATA_LINE_COMM_MODE_7,
                          ORBIS_SPI_CHANNEL);

    /* Configure the McSPI bus clock depending on clock mode */
    /* MCSPI_CLK_MODE_2: Orbis loads data on rising clk edge, read it on the falling
       clock edge; the first bit is loaded when CS goes low */
    McSPIClkConfig(SOC_SPI_0_REGS, MCSPI_IN_CLK, MCSPI_OUT_FREQ, ORBIS_SPI_CHANNEL,
                   MCSPI_CLK_MODE_1);

    /* Configure the word length */
    McSPIWordLengthSet(SOC_SPI_0_REGS, MCSPI_WORD_LENGTH(ORBIS_BITS_PER_WORD), ORBIS_SPI_CHANNEL);

    /* Set polarity of SPIEN (CS) to low */
    McSPICSPolarityConfig(SOC_SPI_0_REGS, MCSPI_CS_POL_LOW, ORBIS_SPI_CHANNEL);

    /* Enable the receiver FIFO of McSPI peripheral */
    McSPIRxFIFOConfig(SOC_SPI_0_REGS, MCSPI_RX_FIFO_ENABLE, ORBIS_SPI_CHANNEL);

    /* Disable the transmitter FIFO of McSPI peripheral */
    McSPITxFIFOConfig(SOC_SPI_0_REGS, MCSPI_TX_FIFO_DISABLE, ORBIS_SPI_CHANNEL);

    /* Set the interrupt levels for the FIFO to avoid RX_FULL interrupts (zero isn't a valid input!) */
    /* Interrupt level must be a multiple of the (rounded up) word size */
    McSPIFIFOTrigLvlSet(SOC_SPI_0_REGS, ORBIS_BYTES_PER_WORD*ORBIS_WORD_COUNT, 1, MCSPI_RX_ONLY_MODE);

    DEBUGPRINT("\t+ Orbis: McSPI configured\r\n");
}

/* ************************************************************************ */
void orbisCaptureStart(void)
{
    /* Make it clear we're not ready to receive */
    orbisBufferReady = 0;

    /* Set the number of words to transfer */
    McSPIWordCountSet(SOC_SPI_0_REGS, ORBIS_WORD_COUNT);

    /* Enable the Tx and End-of-word interrupts of McSPI */
    McSPIIntEnable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL) | MCSPI_INT_RX_FULL(ORBIS_SPI_CHANNEL) | MCSPI_INT_EOWKE);

    /* Manually set the CS */
    GPIOPinWrite(CS_REGS, CS_PIN, GPIO_PIN_LOW);

    /* Wait for 8us */
    waitfor(TIMER_1US*8);

    /* Start the transfer */
    McSPIChannelEnable(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);
}

/* ************************************************************************ */
void orbisMcSPIIsr(void)
{
    unsigned int intCode, i;

    intCode = McSPIIntStatusGet(SOC_SPI_0_REGS);

    while(intCode) {

        /* Check for TX_EMPTY (even in RX_ONLY mode, we must put some data in when asked) */
        if (MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL) == (intCode & MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL))) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL));

            /* Transmit some dummy data */
            McSPITransmitData(SOC_SPI_0_REGS, 0, ORBIS_SPI_CHANNEL);
        }

        if (MCSPI_INT_RX_FULL(ORBIS_SPI_CHANNEL) == (intCode & MCSPI_INT_RX_FULL(ORBIS_SPI_CHANNEL))) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_RX_FULL(ORBIS_SPI_CHANNEL));

            /* Extract the data from the FIFO */
            for (i = 0; i < ORBIS_WORD_COUNT; i++) {
                orbisBuffer[i] = McSPIReceiveData(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL) & ORBIS_BIT_MASK;
            }

            /* Manually set the CS */
            GPIOPinWrite(CS_REGS, CS_PIN, GPIO_PIN_HIGH);

            /* Stop the transfer */
            McSPIChannelDisable(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

            /* Disable the Tx and End-of-word interrupts of McSPI */
            McSPIIntDisable(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL) | MCSPI_INT_RX_FULL(ORBIS_SPI_CHANNEL) | MCSPI_INT_EOWKE);

            /* Signal data ready */
            orbisBufferReady = 1;

            /* Extract data */
            orbisMultiturn = (orbisBuffer[0] << 8) | orbisBuffer[1];
            orbisPosition = (orbisBuffer[2] << 6) | (orbisBuffer[3] >> 2);
            orbisStatus = orbisBuffer[3] & 3;
            orbisCRC = orbisBuffer[4];
            orbisAngle = (float)orbisPosition*ORBIS_RESOLUTION;

        }

        /* Check for end-of-word-count */
        if (MCSPI_INT_EOWKE == (intCode & MCSPI_INT_EOWKE)) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_EOWKE);

        }

        if (MCSPI_INT_TX_EMPTY(0) == (intCode & MCSPI_INT_TX_EMPTY(0)))
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(0));
        if (MCSPI_INT_TX_UNDERFLOW(0) == (intCode & MCSPI_INT_TX_UNDERFLOW(0)))
            McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_UNDERFLOW(0));

        intCode = McSPIIntStatusGet(SOC_SPI_0_REGS);
    }

    /* Call the data handler if necessary */
    if (ad760xBufferReady && orbisBufferReady && (ad760xDataHandler != NULL)) {
        orbisBufferReady = 0;
        ad760xDataHandler();
    }
}
