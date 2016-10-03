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
#include "neon_mathfun.h"
#include <string.h>

/* ************************************************************************ */
/* * Defines ************************************************************** */
/* ************************************************************************ */
#define M_2PI  6.283185307f  /* 2*pi */

#define N_FOURIER_MODES 7  /* N_FOURIER_MODES + 1 must be a multiple of 4 */
#define N_FOURIER_COEFF (2*N_FOURIER_MODES + 2)  /* one will always be zero but it's easier/quicker than handling the special case */
#define N_FOURIER_VEC (N_FOURIER_COEFF/4)
#define N_FOURIER_AVE 5

/* ********************************************************************** */
/* * Types ************************************************************** */
/* ********************************************************************** */


/* ********************************************************************** */
/* * Globals (internal) ************************************************* */
/* ********************************************************************** */
static float time_mod_2pi;
static float time_delta;
static float period_start;  /* signal the start of a new period */
static unsigned int output_channel;  /* DAC output channel */

/* Forcing parameters */
static float forcing_freq;
static float forcing;
static float forcing_coeffs[N_FOURIER_COEFF] MEM_ALIGN;  /* Fourier coeffs of the forcing */

/* Fourier calculations */
static float sinusoid_f[N_FOURIER_COEFF] MEM_ALIGN;  /* [sin(0*t), sin(1*t), sin(2*t), ..., cos(0*t), cos(1*t), cos(2*t), ...] */

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
void calc_sinusoids();

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void rtc_user_init(void)
{
	/* initialise time and other fundamental stuff */
	time_mod_2pi = 0.0f; rtc_data_add_par("time_mod_2pi", &time_mod_2pi, RTC_TYPE_FLOAT, sizeof(time_mod_2pi), NULL, NULL);
	time_delta = M_2PI/TIMER_FREQ;
	period_start = 1;
	output_channel = 0; rtc_data_add_par("output_channel", &output_channel, RTC_TYPE_UINT32, sizeof(output_channel), NULL, NULL);

	/* forcing parameters */
	forcing_freq = 1.0f; rtc_data_add_par("forcing_freq", &forcing_freq, RTC_TYPE_FLOAT, sizeof(forcing_freq), NULL, NULL);
	forcing = 0.0f; rtc_data_add_par("forcing", &forcing, RTC_TYPE_FLOAT, sizeof(forcing), NULL, NULL);
	memset(forcing_coeffs, 0, sizeof(forcing_coeffs)); rtc_data_add_par("forcing_coeffs", &forcing_coeffs, RTC_TYPE_FLOAT, sizeof(forcing_coeffs), NULL, NULL);
	rtc_data_add_par("forcing_amp", &(forcing_coeffs[1]), RTC_TYPE_FLOAT, sizeof(forcing_coeffs[1]), NULL, NULL);
}

/* ************************************************************************ */
void rtc_user_main(void)
{
	static int led = 0;
	static unsigned int time = 0;
	unsigned int i;

	/* ********************************************************************** */
	/*  Pre-calculations */
	/* ********************************************************************** */

	/* calculate sinusoids for use in the Fourier calculations */
	calc_sinusoids();

	/* ********************************************************************** */
	/*  Real-time control */
	/* ********************************************************************** */

	/* calculate the forcing */
	forcing = 0.0f;
	for (i = 0; i < N_FOURIER_COEFF; i++)
		forcing += sinusoid_f[i]*forcing_coeffs[i];

	/* output the calculated value */
	rtc_set_output(output_channel, forcing);

	/* ********************************************************************** */
	/*  Update time */
	/* ********************************************************************** */

	time += 1;
	time_mod_2pi += time_delta*forcing_freq;
	if (time >= SAMPLE_FREQ) {
		time = 0;
		rtc_led(2, led);
		led = !led;
	}

}

/* **********************************************************************
	Calculate sinusoids
   ********************************************************************** */
void calc_sinusoids()
{
	int i;
	v4sf t MEM_ALIGN, sine MEM_ALIGN, cosine MEM_ALIGN;
	float tt[4] MEM_ALIGN = {0.0f, 1.0f, 2.0f, 3.0f};

	t = vld1q_f32(tt);
	t = vmulq_f32(t, vdupq_n_f32(time_mod_2pi));
	sincos_ps(t, &sine, &cosine);
	vst1q_f32(&sinusoid_f[0], sine);
	vst1q_f32(&sinusoid_f[N_FOURIER_COEFF/2], cosine);

	for (i = 1; i < N_FOURIER_COEFF/8; i++) {
		t = vaddq_f32(t, vdupq_n_f32(4.0f*time_mod_2pi));
		sincos_ps(t, &sine, &cosine);
		vst1q_f32(&sinusoid_f[4*i], sine);
		vst1q_f32(&sinusoid_f[4*i + N_FOURIER_COEFF/2], cosine);
	}
}
