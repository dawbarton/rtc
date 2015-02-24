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
#ifndef __DMTIMER_PWM_H__
#define __DMTIMER_PWM_H__

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define DMTIMER_PWM_PT_PULSE           0
#define DMTIMER_PWM_PT_TOGGLE          (1 << 12)
#define DMTIMER_PWM_PT_CLEAR           (1 << 12)
#define DMTIMER_PWM_TRG_NONE           0
#define DMTIMER_PWM_TRG_OVERFLOW       (1 << 10)
#define DMTIMER_PWM_TRG_OVERFLOW_MATCH (2 << 10)
#define DMTIMER_PWM_TRG_CLEAR          (3 << 10)

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES                         
******************************************************************************/
void DMTimerPWMConfigure(unsigned int baseAdd, unsigned int pwmMode);
void DMTimer4PinMux();
void DMTimer5PinMux();
void DMTimer6PinMux();
void DMTimer7PinMux();


#endif
