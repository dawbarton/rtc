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
#ifndef __RTC_USER_H__
#define __RTC_USER_H__

/* ************************************************************************ */
/* * Includes, defines and global variables ******************************* */
/* ************************************************************************ */

#define TIMER_MASTER_FREQ               (24000000) /* main clock at 24 MHz */
#define TIMER_1US                       (0x18)
#define TIMER_10US                      (0xf0)
#define TIMER_100US                     (0x960)
#define TIMER_1MS                       (0x5DC0)
#define TIMER_OVERFLOW                  (0xFFFFFFFFu)
#define TIMER_FREQ                      (1000)  /* Hz - make sure that TIMER_MASTER_FREQ/TIMER_FREQ is an integer */
#define TIMER_PERIOD                    (TIMER_MASTER_FREQ/TIMER_FREQ)
#define TIMER_PERIOD_FLOAT              (1.0f/TIMER_FREQ)
#define TIMER_INITIAL_COUNT             (TIMER_OVERFLOW - TIMER_PERIOD)
#define TIMER_CMP_COUNT                 (TIMER_OVERFLOW - TIMER_PERIOD/2)
#define TIMER_RLD_COUNT                 (TIMER_INITIAL_COUNT)

/* Set to one when the user code has finished executing */
extern volatile unsigned int rtc_user_finished;

/* The input voltages */
extern volatile float in_volt[];

/* Scaling factor for output voltages */

/* ************************************************************************ */
/* * Prototypes *********************************************************** */
/* ************************************************************************ */

void rtc_user_main(void);
void rtc_user_main_handler(void);
void rtc_user_init(void);
void rtc_user_init_handler(void);

void rtc_set_output(unsigned int channel, float value);

#endif
