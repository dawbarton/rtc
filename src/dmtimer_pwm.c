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
/*
** Modified by David AW Barton (2014) to work with the Beaglebone Black.
*/

/******************************************************************************
**                               INCLUDES               
*******************************************************************************/
#include "dmtimer_pwm.h"
#include "dmtimer.h"
#include "hw_types.h"
#include "gpio_setup.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define DMTimerWaitForWrite(reg, baseAdd)   \
            if(HWREG(baseAdd + DMTIMER_TSICR) & DMTIMER_TSICR_POSTED)\
            while((reg & DMTimerWritePostedStatusGet(baseAdd)));

/******************************************************************************
**                      INTERNAL FUNCTION DEFINITIONS                       
******************************************************************************/
/**
 * \brief   This API will configure the timer pulse-width-modulation mode
 *
 * \param   baseAdd      Base Address of the DMTimer Module Register.
 * \param   pwmMode      Mode for pulse-width modulation (PT and TRG registers).
 *
 * 'pwmMode' can take the following values \n
 *     DMTIMER_PWM_PT_PULSE            Pulse mode
 *     DMTIMER_PWM_PT_TOGGLE           Toggle mode
 *     DMTIMER_PWM_TRG_NONE            No trigger
 *     DMTIMER_PWM_TRG_OVERFLOW        Trigger on overflow
 *     DMTIMER_PWM_TRG_OVERFLOW_MATCH  Trigger on overflow or match
 *
 * \return  None.
 *
 **/
void DMTimerPWMConfigure(unsigned int baseAdd, unsigned int pwmMode)
{
    /* Wait for previous write to complete */
    DMTimerWaitForWrite(DMTIMER_WRITE_POST_TCLR, baseAdd);

    /* Clear the PWM configuration */
    HWREG(baseAdd + DMTIMER_TCLR) &= ~(DMTIMER_PWM_PT_CLEAR | DMTIMER_PWM_TRG_CLEAR);

    /* Wait for previous write to complete */
    DMTimerWaitForWrite(DMTIMER_WRITE_POST_TCLR, baseAdd);

    /* Clear the PWM configuration */
    HWREG(baseAdd + DMTIMER_TCLR) |= pwmMode;
}

/*
** Set up pin muxes for the timers
*/
void DMTimer4PinMux()
{
	GPIOPinMuxSetup(GPIO_2_2, PAD_FS_RXD_NA_PUPDD(2));
}

void DMTimer5PinMux()
{
	GPIOPinMuxSetup(GPIO_2_5, PAD_FS_RXD_NA_PUPDD(2));
}

void DMTimer6PinMux()
{
	GPIOPinMuxSetup(GPIO_2_4, PAD_FS_RXD_NA_PUPDD(2));
}

void DMTimer7PinMux()
{
	GPIOPinMuxSetup(GPIO_2_3, PAD_FS_RXD_NA_PUPDD(2));
}
