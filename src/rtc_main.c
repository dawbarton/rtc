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
#include "rtc_main.h"

#include "beaglebone.h"
#include "soc_AM335x.h"
#include "cache.h"
#include "interrupt.h"
#include "dmtimer.h"
#include "dmtimer_pwm.h"

#include "mmu_setup.h"
#include "usb_setup.h"
#include "gpio_setup.h"
#include "ad760x.h"
#include "ad5064.h"

#include "consoleUtils.h"

#include "rtc_messaging.h"
#include "rtc_data.h"
#include "rtc_user.h"
#include "rtc_util.h"

unsigned int time_ctr;

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
static void initialise(void);
static void enable_runfast(void);

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
int main(void)
{
    /* Initialise the hardware */
    initialise();

    /* Initialise the data transfer structures */
    ConsoleUtilsPrintf("Initialising data transfer structures... ");
    rtc_data_init();
    ConsoleUtilsPrintf("done\r\n\r\n");

    /* Initialise user code (indirectly) */
    ConsoleUtilsPrintf("Initialising user code... ");
    rtc_user_init_handler();
    ConsoleUtilsPrintf("done\r\n\r\n");

    /* Start the timer to capture data */
    ConsoleUtilsPrintf("Enabling analogue to digital conversion... ");
    ad760xEnablePWM();
    ConsoleUtilsPrintf("done\r\n\r\n");

    /* Process messages from the host system */
    ConsoleUtilsPrintf("Processing messages from host...\r\n\r\n");
    rtc_handle_messages();

    /* Never gets here */
    return 0;
}


/* ************************************************************************ */
void initialise(void)
{
    /* ******************************************************************** */
    /* * UART ************************************************************* */
    /* ******************************************************************** */

    /* Initialize console for communication with the Host Machine */
    ConsoleUtilsInit();
    ConsoleUtilsSetType(CONSOLE_UART);
    ConsoleUtilsPrintf("\r\n\r\nInitialising hardware...\r\n");

    /* ******************************************************************** */
    /* * Device initialisation ******************************************** */
    /* ******************************************************************** */

    /* Enable the MMU */
    MMUConfigAndEnable();
    ConsoleUtilsPrintf("\t+ MMU configured and enabled\r\n");

    /* Enable caches */
    CacheEnable(CACHE_ALL);
    ConsoleUtilsPrintf("\t+ CPU cache enabled\r\n");

    /* Enable runfast */
    enable_runfast();
    ConsoleUtilsPrintf("\t+ runfast enabled\r\n");

    /* ******************************************************************** */
    /* * Enable interrupts ************************************************ */
    /* ******************************************************************** */

    /* Enable interrupts */
    IntMasterIRQEnable();
    IntAINTCInit();
    ConsoleUtilsPrintf("\t+ interrupts enabled\r\n");

    /* ******************************************************************** */
    /* * USB ************************************************************** */
    /* ******************************************************************** */
    usb_init();

    ConsoleUtilsPrintf("\t+ USB done\r\n");

    /* ******************************************************************** */
    /* * GPIOs ************************************************************ */
    /* ******************************************************************** */

    /* Configure necessary GPIOs */
    GPIO0ModuleClkConfig();
    GPIO1ModuleClkConfig();
    GPIO2ModuleClkConfig();

    /* Enable and reset the GPIO modules */
    GPIOModuleEnable(SOC_GPIO_0_REGS);
    GPIOModuleReset(SOC_GPIO_0_REGS);
    GPIOModuleEnable(SOC_GPIO_1_REGS);
    GPIOModuleReset(SOC_GPIO_1_REGS);
    GPIOModuleEnable(SOC_GPIO_2_REGS);
    GPIOModuleReset(SOC_GPIO_2_REGS);
    ConsoleUtilsPrintf("\t+ GPIOs done\r\n");

    /* Enable user LEDs */
    GPIOPinMuxSetup(GPIO_1_21, PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(GPIO_1_22, PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(GPIO_1_23, PAD_FS_RXD_NA_PUPDD(7));
    GPIOPinMuxSetup(GPIO_1_24, PAD_FS_RXD_NA_PUPDD(7));
    GPIODirModeSet(SOC_GPIO_1_REGS, 21, GPIO_DIR_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 22, GPIO_DIR_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 23, GPIO_DIR_OUTPUT);
    GPIODirModeSet(SOC_GPIO_1_REGS, 24, GPIO_DIR_OUTPUT);

    /* Disable all LEDs */
    rtc_led(0, 0);
    rtc_led(1, 0);
    rtc_led(2, 0);
    rtc_led(3, 0);
    ConsoleUtilsPrintf("\t+ LEDs configured\r\n");

    /* ******************************************************************** */
    /* * General purpose timer ******************************************** */
    /* ******************************************************************** */

    DMTimer4ModuleClkConfig();
    DMTimerPreScalerClkDisable(SOC_DMTIMER_4_REGS);
    DMTimerCounterSet(SOC_DMTIMER_4_REGS, 0);
    DMTimerReloadSet(SOC_DMTIMER_4_REGS, 0);
    DMTimerModeConfigure(SOC_DMTIMER_4_REGS, DMTIMER_AUTORLD_NOCMP_ENABLE);
    DMTimerEnable(SOC_DMTIMER_4_REGS);
    ConsoleUtilsPrintf("\t+ DMTimer4 done\r\n");

    /* ******************************************************************** */
    /* * AD760x *********************************************************** */
    /* ******************************************************************** */

    ad760xSetup();
    ad760xSetupPWM(TIMER_PERIOD);

    /* Set the oversample rate based on the sample frequency */
    if (TIMER_FREQ > 13700)
        ad760xSetOverSample(AD760X_OVERSAMPLE_2);
    else if (TIMER_FREQ > 10300)
        ad760xSetOverSample(AD760X_OVERSAMPLE_4);
    else if (TIMER_FREQ > 6000)
        ad760xSetOverSample(AD760X_OVERSAMPLE_8);
    else if (TIMER_FREQ > 3000)
        ad760xSetOverSample(AD760X_OVERSAMPLE_16);
    else if (TIMER_FREQ > 1500)
        ad760xSetOverSample(AD760X_OVERSAMPLE_32);
    else
        ad760xSetOverSample(AD760X_OVERSAMPLE_64);

    ConsoleUtilsPrintf("\t+ AD760x done\r\n");

    /* ******************************************************************** */
    /* * Done ************************************************************* */
    /* ******************************************************************** */

    rtc_led(0, 1);
    ConsoleUtilsPrintf("\t+ all done!\r\n\r\n");
}

/* ************************************************************************ */
void enable_runfast()
{
    static const unsigned int x = 0x04086060;
    static const unsigned int y = 0x03000000;
    int r;
    asm volatile (
        "fmrx   %0, fpscr           \n\t"   /*r0 = FPSCR */
        "and    %0, %0, %1          \n\t"   /*r0 = r0 & 0x04086060 */
        "orr    %0, %0, %2          \n\t"   /*r0 = r0 | 0x03000000 */
        "fmxr   fpscr, %0           \n\t"   /*FPSCR = r0 */
        : "=r"(r)
        : "r"(x), "r"(y)
    );
}
