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
** ad5064.c
**
** This unit implements an interface to an Analogue Devices AD5064 D/A
** converter. The code below should be easy to modify to work with the
** AD5044 and AD5024 as well.
**
** Usage assumptions/notes:
**   1) There is a 3 usec delay required between transmits; the transmit
**      function simply blocks until enough time has passed.
**   2) Assumes there is enough space in the FIFO to send the required
**      messages; the 3 usec delay should mean this assumption is always
**      satisified.
**   3) A free running timer is preconfigured.
**   4) Relevant GPIO modules are enabled and reset.
**
** Use of global resources:
**   1) Relevant GPIO modules and pins are used.
**   2) A free running 24 MHz timer is used for time outs. (DMTIMER4 is used
**      but can be redefined in the defines section below.)
**   3) The McSPI0 module is used.
*/

/* ************************************************************************ */
/* * Includes, defines and global variables ******************************* */
/* ************************************************************************ */
#include "ad5064.h"
#include "beaglebone.h"
#include "soc_AM335x.h"
#include "mcspi.h"
#include "mcspi_beaglebone.h"
#include "interrupt.h"
#include "gpio_setup.h"
#include "rtc_user.h"
#include "rtc_util.h"

#define AD5064_SPI_CHANNEL 0
#define AD5064_BIT_COUNT 32
#define MCSPI_IN_CLK 48000000
#define MCSPI_OUT_FREQ 48000000

#define LDAC_REGS SOC_GPIO_2_REGS
#define LDAC_PIN 2
#define LDAC_PIN_MUX GPIO_2_2
#define CLR_REGS SOC_GPIO_2_REGS
#define CLR_PIN 3
#define CLR_PIN_MUX GPIO_2_3

#define CMD_SHIFT  24
#define ADDR_SHIFT 20
#define DATA_SHIFT  4
#define DATA_MASK  0xFFFF

#define CMD_WRITE_TO_INPUT_N             0
#define CMD_UPDATE_REG_N                 1
#define CMD_WRITE_TO_INPUT_N_UPDATE_ALL  2
#define CMD_WRITE_TO_INPUT_N_UPDATE_N    3
#define CMD_POWER_UP_DOWN                4
#define CMD_LOAD_CLR_REG                 5
#define CMD_LOAD_LDAC_REG                5
#define CMD_RESET                        7
#define CMD_LOAD_DCEN_REG                8

#define ADDR_CHANNEL_A    0
#define ADDR_CHANNEL_B    1
#define ADDR_CHANNEL_C    2
#define ADDR_CHANNEL_D    3
#define ADDR_CHANNEL_ALL  15

volatile unsigned int ad5064Buffer[4];

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
/* Send data to the AD5064 */
static void ad5064Transmit(unsigned int value);

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void ad5064Setup(void)
{
    /* ******************************************************************* */
    /* McSPI: Initialise clocks and do pin-muxing */

    /* Enable the clocks for McSPI0 module.*/
    McSPI0ModuleClkConfig();

    /* Perform Pin-Muxing for SPI0 Instance: SPI is mode 0 of these pins */
    GPIOPinMuxSetup(CONTROL_CONF_SPI0_SCLK, PAD_FS_RXE_NA_PUPDD(0));
    GPIOPinMuxSetup(CONTROL_CONF_SPI0_D0,   PAD_FS_RXD_NA_PUPDD(0));
    GPIOPinMuxSetup(CONTROL_CONF_SPI0_D1,   PAD_FS_RXE_PU_PUPDE(0));
    GPIOPinMuxSetup(CONTROL_CONF_SPI0_CS0,  PAD_FS_RXD_NA_PUPDD(0));

    /* ******************************************************************* */
    /* McSPI: Set up McSPI */

    /* Reset the McSPI instance */
    McSPIReset(SOC_SPI_0_REGS);

    /* Enable chip select pin */
    McSPICSEnable(SOC_SPI_0_REGS);

    /* Enable master mode of operation */
    McSPIMasterModeEnable(SOC_SPI_0_REGS);

    /* Perform the necessary configuration for master mode */
    /* MCSPI_DATA_LINE_COMM_MODE_6 = D0 enabled for output, D1 disabled for output, receive on D1 */
    McSPIMasterModeConfig(SOC_SPI_0_REGS, MCSPI_MULTI_CH,
                          MCSPI_TX_ONLY_MODE, MCSPI_DATA_LINE_COMM_MODE_6,
                          AD5064_SPI_CHANNEL);

    /* Set D1 to be an input at module level*/
    HWREG(SOC_SPI_0_REGS + MCSPI_SYST) |= (1 << 9);

    /* Configure the McSPI bus clock depending on clock mode */
    /* MCSPI_CLK_MODE_2: AD5064 reads data on the falling clock edge */
    McSPIClkConfig(SOC_SPI_0_REGS, MCSPI_IN_CLK, MCSPI_OUT_FREQ, AD5064_SPI_CHANNEL,
                   MCSPI_CLK_MODE_2);

    /* Configure the word length */
    McSPIWordLengthSet(SOC_SPI_0_REGS, MCSPI_WORD_LENGTH(AD5064_BIT_COUNT), AD5064_SPI_CHANNEL);

    /* Set polarity of SPIEN (CS) to low */
    McSPICSPolarityConfig(SOC_SPI_0_REGS, MCSPI_CS_POL_LOW, AD5064_SPI_CHANNEL);

    /* Disable the receiver FIFO of McSPI peripheral */
    McSPIRxFIFOConfig(SOC_SPI_0_REGS, MCSPI_RX_FIFO_DISABLE, AD5064_SPI_CHANNEL);

    /* Disable the transmitter FIFO of McSPI peripheral */
    McSPITxFIFOConfig(SOC_SPI_0_REGS, MCSPI_TX_FIFO_DISABLE, AD5064_SPI_CHANNEL);

    /* Disable any send delays */
    McSPIInitDelayConfig(SOC_SPI_0_REGS, MCSPI_INITDLY_0);

    /* Set the timing of the CS assertion to be minimal but allowable at 48 MHz */
    McSPICSTimeControlSet(SOC_SPI_0_REGS, MCSPI_CS_TCS_1PNT5_CLK, AD5064_SPI_CHANNEL);

    /* Enable the McSPI channel for communication - CS is automatically asserted in multi channel mode */
    McSPIChannelEnable(SOC_SPI_0_REGS, AD5064_SPI_CHANNEL);


    /* ******************************************************************* */
    /* GPIOs: Set up GPIOs */

    /* Pin muxes for necessary pins */
    GPIOPinMuxSetup(LDAC_PIN_MUX,   PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(CLR_PIN_MUX,    PAD_FS_RXD_NA_PUPDD(7));

    /* Pin directions */
    GPIODirModeSet(LDAC_REGS,   LDAC_PIN,   GPIO_DIR_OUTPUT);
    GPIODirModeSet(CLR_REGS,    CLR_PIN,    GPIO_DIR_OUTPUT);

    /* Set default outputs */
    GPIOPinWrite(LDAC_REGS,   LDAC_PIN,   GPIO_PIN_LOW); /* Synchronous update by default */
    GPIOPinWrite(CLR_REGS,    CLR_PIN,    GPIO_PIN_HIGH);

    /* ******************************************************************* */
    /* Initialise the device */

    /* Reset the AD5064 */
    ad5064Reset();

    /* Initialise the buffers with default values (may not be correct!) */
    ad5064Buffer[0] = 0;
    ad5064Buffer[1] = 0;
    ad5064Buffer[2] = 0;
    ad5064Buffer[3] = 0;
}

/* ************************************************************************ */
void ad5064Reset(void)
{
    ad5064Transmit(CMD_RESET << CMD_SHIFT);
}

/* ************************************************************************ */
void ad5064SetOutput(unsigned int channel, unsigned int value)
{
    if (channel > 3)
        return;
    ad5064Buffer[channel] = value;
    ad5064Transmit((CMD_WRITE_TO_INPUT_N_UPDATE_N << CMD_SHIFT) | (channel << ADDR_SHIFT) | ((value & DATA_MASK) << DATA_SHIFT));
}

/* ************************************************************************ */
void ad5064Transmit(unsigned int value)
{
    static unsigned int last_transmit = 0;
    /* Use 4 usec delay as 3 usec isn't always enough */
    while ((TIME - last_transmit) < (4*TIMER_1US));
    McSPITransmitData(SOC_SPI_0_REGS, value, AD5064_SPI_CHANNEL);
    last_transmit = TIME;
}
