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

/* ************************************************************************
	Notes:

	NEON
	====

	Remember to make sure that all variables that get used in NEON intrinsics
	(i.e., vectorised floating point operations) are memory aligned with
	__attribute__((__aligned__(16))). Generally this just means float arrays.

	Note, this has to be applied to all the variables individually; i.e.
		v4sf t, sine, cosine __attribute__((__aligned__(16)));
	does not work as you might expect. Instead use
		v4sf t __attribute__((__aligned__(16))), sine __attribute__((__aligned__(16))), cosine __attribute__((__aligned__(16)));

	Macros are your friend...
		v4sf t MEM_ALIGN, sine MEM_ALIGN, cosine MEM_ALIGN;

	AD5064
	======

	Sequential writes to the AD5064 are slow due to the requirement of 3us
	between each word transmitted. As such, try to space out writes so that
	execution isn't needlessly waiting for the 3us to elapse.

   ************************************************************************ */

/* ************************************************************************ */
/* * Includes, defines and global variables ******************************* */
/* ************************************************************************ */
#include "rtc_data.h"
#include "rtc_main.h"
#include "rtc_util.h"
#include "rtc_user.h"

/* ************************************************************************ */
/* * Defines ************************************************************** */
/* ************************************************************************ */


/* ********************************************************************** */
/* * Types ************************************************************** */
/* ********************************************************************** */


/* ********************************************************************** */
/* * Globals (internal) ************************************************* */
/* ********************************************************************** */


/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */


/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void rtc_user_init(void)
{
}

/* ************************************************************************ */
void rtc_user_main(void)
{
	static int led = 0;
	static unsigned int time = 0;

	/* ********************************************************************** */
	/*  Update time */
	/* ********************************************************************** */

	time += 1;
	if (time >= SAMPLE_FREQ) {
		time = 0;
		rtc_led(2, led);
		led = !led;
	}
}
