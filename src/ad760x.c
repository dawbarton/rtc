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
** ad760x.c
**
** This unit implements an interface to an Analogue Devices AD760x A/D
** converter.
**
** Usage assumptions/notes:
**   1) Conversion start can be either triggered manually using the 
**      ad760xCaptureStart function or via setting up pulse-width modulation
**      via the DMTimer module.
**   2) Getting the data from the device occurs automatically via interrupts.
**   3) A free running timer is preconfigured.
**   4) Relevant GPIO modules are enabled and reset.
**
** Use of global resources:
**   1) Relevant GPIO modules and pins are used.
**   2) A free running 24 MHz timer is used for time outs. (DMTIMER4 is used
**      but can be redefined in the defines section below.)
**   3) DMTIMER6 is used for timer-based conversion triggering.
**   4) Interrupt handler SYS_INT_GPIOINT1A is used.
**   5) The McSPI1 module is used.
*/

/* ************************************************************************ */
/* * Includes, defines and global variables ******************************* */
/* ************************************************************************ */
#include "ad760x.h"
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

#define AD760X_SPI_CHANNEL 0
#define MCSPI_IN_CLK 48000000
#define MCSPI_OUT_FREQ 12000000

#define OS0_REGS SOC_GPIO_2_REGS
#define OS0_PIN 1
#define OS0_PIN_MUX GPIO_2_1
#define OS1_REGS SOC_GPIO_0_REGS
#define OS1_PIN 27
#define OS1_PIN_MUX GPIO_0_27
#define OS2_REGS SOC_GPIO_1_REGS
#define OS2_PIN 14
#define OS2_PIN_MUX GPIO_1_14
#define STBY_REGS SOC_GPIO_1_REGS
#define STBY_PIN 15
#define STBY_PIN_MUX GPIO_1_15
#define RANGE_REGS SOC_GPIO_0_REGS
#define RANGE_PIN 26
#define RANGE_PIN_MUX GPIO_0_26
#define CONVST_REGS SOC_GPIO_2_REGS
#define CONVST_PIN 4
#define CONVST_PIN_MUX GPIO_2_4
#define RESET_REGS SOC_GPIO_1_REGS
#define RESET_PIN 12
#define RESET_PIN_MUX GPIO_1_12
#define BUSY_REGS SOC_GPIO_1_REGS
#define BUSY_PIN 13
#define BUSY_PIN_MUX GPIO_1_13
#define BUSY_INT SYS_INT_GPIOINT1A

#define CONTROL_CONF_SPI1_SCLK GPIO_3_14
#define CONTROL_CONF_SPI1_CS0  GPIO_3_17
#define CONTROL_CONF_SPI1_D0   GPIO_3_15
#define CONTROL_CONF_SPI1_D1   GPIO_3_16

#define AD760X_SIGN_MASK   (1 << (AD760X_BIT_COUNT - 1))
#define AD760X_SIGN_EXT    (0xFFFF << AD760X_BIT_COUNT)

volatile int ad760xBuffer[AD760X_CHANNEL_COUNT];
volatile int ad760xBufferReady = 0;
void (*ad760xDataHandler)(void);

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
/* Wait for a certain number of counter ticks */
static void waitfor(unsigned int);
/* Get conversion data from the device */
static void ad760xCaptureGet(void);
/* The McSPI interrupt handler */
static void ad760xMcSPIIsr(void);
/* The GPIO interrupt handler */
static void ad760xGPIOIsr(void);

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void waitfor(unsigned int duration)
{
    unsigned int t0 = TIME;
    while ((TIME - t0) < duration);
}

/* ************************************************************************ */
void ad760xSetup(void)
{
    /* ******************************************************************* */
    /* Initialise the data handling routine */
    ad760xDataHandler = NULL;

    /* ******************************************************************* */
    /* McSPI: Initialise clocks and do pin-muxing */

    /* Enable the clocks for McSPI1 module.*/
    McSPI1ModuleClkConfig();

    DEBUGPRINT("\t+ AD760x: enabled clocks for SPI1\r\n");

    /* Perform Pin-Muxing for SPI1 Instance: SPI is mode 3 of these pins */
    GPIOPinMuxSetup(CONTROL_CONF_SPI1_SCLK, PAD_FS_RXE_NA_PUPDD(3));
    GPIOPinMuxSetup(CONTROL_CONF_SPI1_D0,   PAD_FS_RXD_NA_PUPDD(3));
    GPIOPinMuxSetup(CONTROL_CONF_SPI1_D1,   PAD_FS_RXE_PU_PUPDE(3));
    GPIOPinMuxSetup(CONTROL_CONF_SPI1_CS0,  PAD_FS_RXD_NA_PUPDD(3));

    DEBUGPRINT("\t+ AD760x: done pin-muxing\r\n");

    /* ******************************************************************* */
    /* McSPI: Sort out interrupts */

    /* Register McSPIIsr interrupt handler */
	IntRegister(SYS_INT_SPI1INT, ad760xMcSPIIsr);

	/* Set Interrupt Priority */
	IntPrioritySet(SYS_INT_SPI1INT, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable system interrupt in AINTC */
	IntSystemEnable(SYS_INT_SPI1INT);

    DEBUGPRINT("\t+ AD760x: interrupts for McSPI registered and enabled\r\n");

    /* ******************************************************************* */
    /* McSPI: Set up McSPI */

    /* Reset the McSPI instance */
    McSPIReset(SOC_SPI_1_REGS);

    /* Enable chip select pin */
    McSPICSEnable(SOC_SPI_1_REGS);

    /* Enable master mode of operation */
    McSPIMasterModeEnable(SOC_SPI_1_REGS);

    /* Perform the necessary configuration for master mode */
    /* MCSPI_DATA_LINE_COMM_MODE_7 = D0 & D1 disabled for output, receive on D1 */
    McSPIMasterModeConfig(SOC_SPI_1_REGS, MCSPI_SINGLE_CH, 
                          MCSPI_RX_ONLY_MODE, MCSPI_DATA_LINE_COMM_MODE_7,
                          AD760X_SPI_CHANNEL);

    /* Set D1 to be an input at module level*/
    HWREG(SOC_SPI_1_REGS + MCSPI_SYST) |= (1 << 9);

    /* Configure the McSPI bus clock depending on clock mode */
    /* MCSPI_CLK_MODE_2: AD760X loads data on rising clk edge, read it on the falling 
       clock edge; the first bit is loaded when CS goes low */
    McSPIClkConfig(SOC_SPI_1_REGS, MCSPI_IN_CLK, MCSPI_OUT_FREQ, AD760X_SPI_CHANNEL, 
                   MCSPI_CLK_MODE_2);

    /* Configure the word length */
    McSPIWordLengthSet(SOC_SPI_1_REGS, MCSPI_WORD_LENGTH(AD760X_BIT_COUNT), AD760X_SPI_CHANNEL);

    /* Set polarity of SPIEN (CS) to low */
    McSPICSPolarityConfig(SOC_SPI_1_REGS, MCSPI_CS_POL_LOW, AD760X_SPI_CHANNEL);

    /* Enable the receiver FIFO of McSPI peripheral */
    McSPIRxFIFOConfig(SOC_SPI_1_REGS, MCSPI_RX_FIFO_ENABLE, AD760X_SPI_CHANNEL);

    /* Disable the transmitter FIFO of McSPI peripheral */
    McSPITxFIFOConfig(SOC_SPI_1_REGS, MCSPI_TX_FIFO_DISABLE, AD760X_SPI_CHANNEL);

    /* Set the interrupt levels for the FIFO to avoid RX_FULL interrupts (zero isn't a valid input!) */
    /* Interrupt level must be a multiple of the (rounded up) word size */
    McSPIFIFOTrigLvlSet(SOC_SPI_1_REGS, AD760X_BYTE_COUNT*AD760X_CHANNEL_COUNT, 1, MCSPI_RX_ONLY_MODE);

    DEBUGPRINT("\t+ AD760x: McSPI configured\r\n");

    /* ******************************************************************* */
    /* GPIOs: Set up GPIOs */

    /* Pin muxes for necessary pins */
    GPIOPinMuxSetup(OS0_PIN_MUX,    PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(OS1_PIN_MUX,    PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(OS2_PIN_MUX,    PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(STBY_PIN_MUX,   PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(RANGE_PIN_MUX,  PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(CONVST_PIN_MUX, PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(RESET_PIN_MUX,  PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(BUSY_PIN_MUX,   PAD_FS_RXE_PD_PUPDE(7));
    
    /* Pin directions */
    GPIODirModeSet(OS0_REGS,    OS0_PIN,    GPIO_DIR_OUTPUT);
    GPIODirModeSet(OS1_REGS,    OS1_PIN,    GPIO_DIR_OUTPUT);
    GPIODirModeSet(OS2_REGS,    OS2_PIN,    GPIO_DIR_OUTPUT);
    GPIODirModeSet(STBY_REGS,   STBY_PIN,   GPIO_DIR_OUTPUT);
    GPIODirModeSet(RANGE_REGS,  RANGE_PIN,  GPIO_DIR_OUTPUT);
    GPIODirModeSet(CONVST_REGS, CONVST_PIN, GPIO_DIR_OUTPUT);
    GPIODirModeSet(RESET_REGS,  RESET_PIN,  GPIO_DIR_OUTPUT);
    GPIODirModeSet(BUSY_REGS,   BUSY_PIN,   GPIO_DIR_INPUT);

    /* Set default outputs */
    GPIOPinWrite(OS0_REGS,    OS0_PIN,    GPIO_PIN_LOW);
    GPIOPinWrite(OS1_REGS,    OS1_PIN,    GPIO_PIN_LOW);
    GPIOPinWrite(OS2_REGS,    OS2_PIN,    GPIO_PIN_LOW);
    GPIOPinWrite(STBY_REGS,   STBY_PIN,   GPIO_PIN_LOW);
    GPIOPinWrite(RANGE_REGS,  RANGE_PIN,  GPIO_PIN_LOW);
    GPIOPinWrite(CONVST_REGS, CONVST_PIN, GPIO_PIN_LOW);
    GPIOPinWrite(RESET_REGS,  RESET_PIN,  GPIO_PIN_LOW);

    DEBUGPRINT("\t+ AD760x: GPIOs configured\r\n");

    /* ******************************************************************* */
    /* GPIOs: Set up interrupts for the busy signal */

    /* Register the Interrupt Service Routine(ISR) */
    IntRegister(SYS_INT_GPIOINT1A, ad760xGPIOIsr);

    /* Set the priority for the GPIO0 system interrupt in INTC (lower priority than the SPI handler) */
    IntPrioritySet(SYS_INT_GPIOINT1A, 16, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the GPIO0 system interrupt in INTC */
    IntSystemEnable(SYS_INT_GPIOINT1A);

    /* Disable level detection */
    GPIOIntTypeSet(BUSY_REGS, BUSY_PIN, GPIO_INT_TYPE_NO_LEVEL);

    /* Enable edge detection */
    GPIOIntTypeSet(BUSY_REGS, BUSY_PIN, GPIO_INT_TYPE_FALL_EDGE);

    /* Disable the interrupt for the busy signal */
    GPIOPinIntDisable(BUSY_REGS, GPIO_INT_LINE_1, BUSY_PIN);

    DEBUGPRINT("\t+ AD760x: GPIO interrupts registered\r\n");

    /* ******************************************************************* */
    /* Initialise the device */

    /* Wait for 100ms before powering up the AD760x */
    waitfor(100*TIMER_1MS);

    /* Turn on the AD760x */
    GPIOPinWrite(STBY_REGS, STBY_PIN, GPIO_PIN_HIGH);

    DEBUGPRINT("\t+ AD760x: device switched on\r\n");

    /* Wait for 100ms before resetting the AD760x */
    waitfor(100*TIMER_1MS);

    /* Reset the AD760X */
    ad760xReset();

    DEBUGPRINT("\t+ AD760x: device reset\r\n");  

    /* Enable the interrupt for the busy signal */
    GPIOPinIntEnable(BUSY_REGS, GPIO_INT_LINE_1, BUSY_PIN);

    DEBUGPRINT("\t+ AD760x: conversion complete interrupt enabled\r\n");
}

/* ************************************************************************ */
void ad760xSetupPWM(unsigned int period)
{
    /* Enable clocks for DMTimer6 */
    DMTimer6ModuleClkConfig();

    /* Do the pin mux for DMTimer6 */
    DMTimer6PinMux();

    /* Perform the necessary configurations for DMTimer6 */
    DMTimerDisable(SOC_DMTIMER_6_REGS);
    DMTimerPreScalerClkDisable(SOC_DMTIMER_6_REGS);
    DMTimerCounterSet(SOC_DMTIMER_6_REGS, 0xFFFFFFFFu - period);
    DMTimerReloadSet(SOC_DMTIMER_6_REGS, 0xFFFFFFFFu - period);
    DMTimerCompareSet(SOC_DMTIMER_6_REGS, 0xFFFFFFFFu - period/2);
    DMTimerModeConfigure(SOC_DMTIMER_6_REGS, DMTIMER_AUTORLD_CMP_ENABLE);
    DMTimerPWMConfigure(SOC_DMTIMER_6_REGS, DMTIMER_PWM_PT_TOGGLE | DMTIMER_PWM_TRG_OVERFLOW_MATCH);
}

/* ************************************************************************ */
void ad760xEnablePWM()
{
    /* Enable the timer */
    DMTimerEnable(SOC_DMTIMER_6_REGS);  
}

/* ************************************************************************ */
void ad760xDisablePWM()
{
    /* Enable the timer */
    DMTimerDisable(SOC_DMTIMER_6_REGS);  
}

/* ************************************************************************ */
void ad760xReset(void)
{
    /* Reset the buffer status */
    ad760xBufferReady = 0;

    /* Set the reset pin high */
    GPIOPinWrite(RESET_REGS,  RESET_PIN,  GPIO_PIN_HIGH);

    /* Wait for 10us (specs say 100ns minimum) */
    waitfor(10*TIMER_1US);

    /* Set the reset pin low */
    GPIOPinWrite(RESET_REGS,  RESET_PIN,  GPIO_PIN_LOW);
}

/* ************************************************************************ */
void ad760xCaptureStart(void)
{
    /* Reset the buffer status */
    ad760xBufferReady = 0;

    /* Initiate conversion */
    GPIOPinWrite(CONVST_REGS, CONVST_PIN, GPIO_PIN_HIGH);
}

/* ************************************************************************ */
void ad760xCaptureGet(void)
{
    /* Set the number of words to transfer */
    McSPIWordCountSet(SOC_SPI_1_REGS, AD760X_CHANNEL_COUNT);

    /* Enable the Tx and End-of-word-count interrupts of McSPI */
    McSPIIntEnable(SOC_SPI_1_REGS, MCSPI_INT_TX_EMPTY(AD760X_SPI_CHANNEL) | MCSPI_INT_RX_FULL(AD760X_SPI_CHANNEL) | MCSPI_INT_EOWKE);

    /* SPIEN (CS) line is forced to low state */
    McSPICSAssert(SOC_SPI_1_REGS, AD760X_SPI_CHANNEL);

    /* Enable the McSPI channel for communication */
    McSPIChannelEnable(SOC_SPI_1_REGS, AD760X_SPI_CHANNEL);
}

/* ************************************************************************ */
void ad760xMcSPIIsr(void)
{
    unsigned int intCode, i, j;

    intCode = McSPIIntStatusGet(SOC_SPI_1_REGS);

    while(intCode) {

        /* Check for TX_EMPTY (even in RX_ONLY mode, we must put some data in when asked) */
        if (MCSPI_INT_TX_EMPTY(AD760X_SPI_CHANNEL) == (intCode & MCSPI_INT_TX_EMPTY(AD760X_SPI_CHANNEL))) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_1_REGS, MCSPI_INT_TX_EMPTY(AD760X_SPI_CHANNEL));

            /* Transmit some dummy data */
            McSPITransmitData(SOC_SPI_1_REGS, 0, AD760X_SPI_CHANNEL);
        }

        if (MCSPI_INT_RX_FULL(AD760X_SPI_CHANNEL) == (intCode & MCSPI_INT_RX_FULL(AD760X_SPI_CHANNEL))) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_1_REGS, MCSPI_INT_RX_FULL(AD760X_SPI_CHANNEL));

            /* Extract the data from the FIFO */
            for (i = 0; i < AD760X_CHANNEL_COUNT; i++) {
                j = McSPIReceiveData(SOC_SPI_1_REGS, AD760X_SPI_CHANNEL) & AD760X_BIT_MASK;
                ad760xBuffer[i] = (j & AD760X_SIGN_MASK) ? (j | AD760X_SIGN_EXT) : j;
            }

            /* Disable the McSPI channel */
            McSPIChannelDisable(SOC_SPI_1_REGS, AD760X_SPI_CHANNEL);

            /* Force SPIEN (CS) line to the inactive state */
            McSPICSDeAssert(SOC_SPI_1_REGS, AD760X_SPI_CHANNEL);

            /* Disable interrupts */
            McSPIIntDisable(SOC_SPI_1_REGS, MCSPI_INT_TX_EMPTY(AD760X_SPI_CHANNEL) | MCSPI_INT_RX_FULL(AD760X_SPI_CHANNEL) | MCSPI_INT_EOWKE);

        }

        /* Check for end-of-word-count */
        if (MCSPI_INT_EOWKE == (intCode & MCSPI_INT_EOWKE)) {

            /* Clear the interrupt status */
            McSPIIntStatusClear(SOC_SPI_1_REGS, MCSPI_INT_EOWKE);

        }

        intCode = McSPIIntStatusGet(SOC_SPI_1_REGS);
    }
}

/* ************************************************************************ */
void ad760xGPIOIsr(void)
{
    /* Check the Interrupt Status of the GPIO Card Detect pin. */
    if(GPIOPinIntStatus(BUSY_REGS, GPIO_INT_LINE_1, BUSY_PIN) & (1 << BUSY_PIN)) {

        /* Clear the Interrupt Status of the GPIO Card Detect pin. */
        GPIOPinIntClear(BUSY_REGS, GPIO_INT_LINE_1, BUSY_PIN);

        /* Set the start conversion signal low again */
        GPIOPinWrite(CONVST_REGS, CONVST_PIN, GPIO_PIN_LOW);

        /* Get the data */
        ad760xCaptureGet();

        /* Signal data ready */
        ad760xBufferReady = 1;

        /* Call the data handler if necessary */
        if (ad760xDataHandler != NULL)
            ad760xDataHandler();
    }
}

/* ************************************************************************ */
void ad760xSetRange(ad760x_range range)
{
    if (range == AD760X_RANGE_HIGH)
        GPIOPinWrite(RANGE_REGS, RANGE_PIN, GPIO_PIN_HIGH);
    else if (range == AD760X_RANGE_LOW)
        GPIOPinWrite(RANGE_REGS, RANGE_PIN, GPIO_PIN_LOW);
}

/* ************************************************************************ */
void ad760xSetOverSample(ad760x_oversample oversample)
{
    switch (oversample) {
        case AD760X_OVERSAMPLE_OFF:
            GPIOPinWrite(OS0_REGS, OS0_PIN, GPIO_PIN_LOW);
            GPIOPinWrite(OS1_REGS, OS1_PIN, GPIO_PIN_LOW);
            GPIOPinWrite(OS2_REGS, OS2_PIN, GPIO_PIN_LOW);
            break;
        case AD760X_OVERSAMPLE_2:
            GPIOPinWrite(OS0_REGS, OS0_PIN, GPIO_PIN_HIGH);
            GPIOPinWrite(OS1_REGS, OS1_PIN, GPIO_PIN_LOW);
            GPIOPinWrite(OS2_REGS, OS2_PIN, GPIO_PIN_LOW);
            break;
        case AD760X_OVERSAMPLE_4:
            GPIOPinWrite(OS0_REGS, OS0_PIN, GPIO_PIN_LOW);
            GPIOPinWrite(OS1_REGS, OS1_PIN, GPIO_PIN_HIGH);
            GPIOPinWrite(OS2_REGS, OS2_PIN, GPIO_PIN_LOW);
            break;
        case AD760X_OVERSAMPLE_8:
            GPIOPinWrite(OS0_REGS, OS0_PIN, GPIO_PIN_HIGH);
            GPIOPinWrite(OS1_REGS, OS1_PIN, GPIO_PIN_HIGH);
            GPIOPinWrite(OS2_REGS, OS2_PIN, GPIO_PIN_LOW);
            break;
        case AD760X_OVERSAMPLE_16:
            GPIOPinWrite(OS0_REGS, OS0_PIN, GPIO_PIN_LOW);
            GPIOPinWrite(OS1_REGS, OS1_PIN, GPIO_PIN_LOW);
            GPIOPinWrite(OS2_REGS, OS2_PIN, GPIO_PIN_HIGH);
            break;
        case AD760X_OVERSAMPLE_32:
            GPIOPinWrite(OS0_REGS, OS0_PIN, GPIO_PIN_HIGH);
            GPIOPinWrite(OS1_REGS, OS1_PIN, GPIO_PIN_LOW);
            GPIOPinWrite(OS2_REGS, OS2_PIN, GPIO_PIN_HIGH);
            break;
        case AD760X_OVERSAMPLE_64:
            GPIOPinWrite(OS0_REGS, OS0_PIN, GPIO_PIN_LOW);
            GPIOPinWrite(OS1_REGS, OS1_PIN, GPIO_PIN_HIGH);
            GPIOPinWrite(OS2_REGS, OS2_PIN, GPIO_PIN_HIGH);
            break;
    }
}
