/*
** Copyright (c) 2018, Irene Tartaruga
** (irene.tartaruga@bristol.ac.uk) All rights reserved.
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
#include "aksim.h"
#include "neon_mathfun.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h> 

/* ********************************************************************** 
	Defines
   ********************************************************************** */
#define M_PI   3.141592654f  /* pi */
#define M_2PI  6.283185307f  /* 2*pi */

#define M_PI_2 1.570796327f  /* pi/2 */
#define M_PI_4 0.785398163f  /* pi/4 */

#define M_1_PI 0.318309886f  /* 1/pi */
#define M_2_PI 0.159154943f  /* 1/(2*pi) */

#define N_FOURIER_MODES 7  /* N_FOURIER_MODES + 1 must be a multiple of 4 */
#define N_FOURIER_COEFF (2*N_FOURIER_MODES + 2)  /* one will always be zero but it's easier/quicker than handling the special case */
#define N_FOURIER_VEC (N_FOURIER_COEFF/4)
#define N_FOURIER_AVE 5

#define INPUT_FILTER_ORDER 2
#define INPUT_FILTER_N_STATE (2*INPUT_FILTER_ORDER)
#define RAND_FILTER_ORDER 3
#define RAND_FILTER_N_STATE (2*RAND_FILTER_ORDER)

/* ********************************************************************** 
	Types
   ********************************************************************** */
struct fourier_t {
	float last_value[N_FOURIER_COEFF];
	float integrator[N_FOURIER_COEFF];
};

struct biquad_filter_t {
	int order;
	float a[2];
	float b[3];
};

/* ********************************************************************** 
	Globals (internal)
   ********************************************************************** */
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

/* Variables relating to the input filternooi */
static float input_filter_freq;
static float input_filter_freq_x1d;
static struct biquad_filter_t input_filter;

/* Variables relating to the force at the shaker position */
static float Fshaker;  /* Force Shaker */


/* Variables relating to the relative heave position (x1)*/
static float phi, phi_bis, time_star;  /* angle and additional variable used to check that the angle is well calculated */
static float x1, x1_raw, x1d, x1d_raw, x1_last;  /* heave position */
static float x1_target, x1d_target;  /* Target for x1 (set by the Fourier coefficients) */
static float x1_filter_state[INPUT_FILTER_N_STATE];  /* Filtering for the error in x1 */
static float x1d_filter_state[INPUT_FILTER_N_STATE];  /* Filtering for the error in x1 */
static float x1_target_filter_state[INPUT_FILTER_N_STATE];  /* Filtering for the error in x1 */
static float x1d_target_filter_state[INPUT_FILTER_N_STATE];  /* Filtering for the error in x1 */
static float x1_error_filter_state[INPUT_FILTER_N_STATE];  /* Filtering for the error in x1 */
static float x1_error_dt_filter_state[INPUT_FILTER_N_STATE];  /* Filtering for the derivative error in x1 */
static float x1_error_raw, x1_error, x1_error_last, x1_error_dt_raw, x1_error_dt;  /* Control error */
static float x1_target_coeffs[N_FOURIER_COEFF] MEM_ALIGN;  /* Fourier coeffs of the x1_target */
static float mean_h; /*approximated mean of the function */

static float x_Kp;  /* Proportional control gain */
static float x_Kd;  /* Derivative control gain */
static int x1_control;  /* Whether control is switched on or off */

static struct fourier_t x1_fourier;  /* Internal structure for Fourier estimation x1 */
static float x1_coeffs[N_FOURIER_COEFF] MEM_ALIGN;  /* Fourier coeffs of x1 */
static float x1_coeffs_ave[N_FOURIER_COEFF] MEM_ALIGN;  /* Average Fourier coeffs of x1 */
static float x1_coeffs_var[N_FOURIER_COEFF] MEM_ALIGN;  /* Variance of Fourier coeffs of x1 */
static float x1_coeffs_arr[N_FOURIER_AVE][N_FOURIER_COEFF] MEM_ALIGN;  /* Last N Fourier coeffs of x1 */
static int x1_coeffs_idx;


/* Variables relating to the output */
static float out;
static struct fourier_t out_fourier;  /* Internal structure for Fourier estimation */
static float out_coeffs[N_FOURIER_COEFF] MEM_ALIGN;  /* Fourier coeffs of the output (channel 1) */
static float out_coeffs_ave[N_FOURIER_COEFF] MEM_ALIGN;  /* Average Fourier coeffs of the output (channel 1) */
static float out_coeffs_var[N_FOURIER_COEFF] MEM_ALIGN;  /* Variance coeffs of the output (channel 1) */
static float out_coeffs_arr[N_FOURIER_AVE][N_FOURIER_COEFF] MEM_ALIGN;  /* Last N Fourier coeffs of the output (channel 1) */
static int out_coeffs_idx;

/* Variables relating to the random input */
static float rand_ampl;  /* amplitude of random input to the control x_target */
static unsigned int rand_seed_w;  /* must not be zero, nor 0x464fffff */
static unsigned int rand_seed_z;  /* must not be zero, nor 0x9068ffff */
static float rand_filter_freq;
static struct biquad_filter_t rand_filter;
static float rand_filter_state[2*RAND_FILTER_ORDER];

/* Other inputs */
/* static float z;

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
void calc_sinusoids();
void update_fourier(float value_f, float output_f[N_FOURIER_COEFF], struct fourier_t *fourier);
float biquad_filter(float x, const struct biquad_filter_t *filter, float state[]);
void update_filter(struct biquad_filter_t *filter, float freq);
void update_input_filter(void *new_value, void *trigger_data);
void update_rand_filter(void *new_value, void *trigger_data);
float inv_tan_pi(float x);
unsigned int get_random();


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

	/* input filter cut-off frequency */
	input_filter_freq = 0.025f; rtc_data_add_par("input_filter_freq", &input_filter_freq, RTC_TYPE_FLOAT, sizeof(input_filter_freq), update_input_filter, NULL);
	input_filter_freq_x1d = 0.025f; rtc_data_add_par("input_filter_freq_x1d", &input_filter_freq_x1d, RTC_TYPE_FLOAT, sizeof(input_filter_freq_x1d), update_input_filter, NULL);
	memset(&input_filter, 0, sizeof(input_filter));

	/* variables relating to the force at the shaker */	
	Fshaker = 0.0; rtc_data_add_par("Fshaker", &Fshaker, RTC_TYPE_FLOAT, sizeof(Fshaker), NULL, NULL);

	/* variables relating to the relative  heave position (x1) */
    phi = 0.0f; rtc_data_add_par("phi", &phi, RTC_TYPE_FLOAT, sizeof(phi), NULL, NULL);
    phi_bis = 0.0f; rtc_data_add_par("phi_bis", &phi_bis, RTC_TYPE_FLOAT, sizeof(phi_bis), NULL, NULL);
    time_star = 0.0f; rtc_data_add_par("time_star", &time_star, RTC_TYPE_FLOAT, sizeof(time_star), NULL, NULL);
    x1 = 0.0f; rtc_data_add_par("x1", &x1, RTC_TYPE_FLOAT, sizeof(x1), NULL, NULL);
    x1d = 0.0f; rtc_data_add_par("x1d", &x1d, RTC_TYPE_FLOAT, sizeof(x1d), NULL, NULL);
    x1_last = 0.0f; 
	x1_raw = 0.0f; rtc_data_add_par("x1_raw", &x1_raw, RTC_TYPE_FLOAT, sizeof(x1_raw), NULL, NULL);
	x1d_raw = 0.0f; rtc_data_add_par("x1d_raw", &x1d_raw, RTC_TYPE_FLOAT, sizeof(x1d_raw), NULL, NULL);
	x1_target = 0.0f; rtc_data_add_par("x1_target", &x1_target, RTC_TYPE_FLOAT, sizeof(x1_target), NULL, NULL);
    x1d_target = 0.0f; rtc_data_add_par("x1d_target", &x1d_target, RTC_TYPE_FLOAT, sizeof(x1d_target), NULL, NULL);
	x1_error_raw = 0.0f; rtc_data_add_par("x1_error_raw", &x1_error_raw, RTC_TYPE_FLOAT, sizeof(x1_error_raw), NULL, NULL);
	x1_error_dt_raw = 0.0f; rtc_data_add_par("x1_error_dt_raw", &x1_error_dt_raw, RTC_TYPE_FLOAT, sizeof(x1_error_dt_raw), NULL, NULL);
	x1_error = 0.0f; rtc_data_add_par("x1_error", &x1_error, RTC_TYPE_FLOAT, sizeof(x1_error), NULL, NULL);
	x1_error_last = 0.0f;
	x1_error_dt = 0.0f; rtc_data_add_par("x1_error_dt", &x1_error_dt, RTC_TYPE_FLOAT, sizeof(x1_error_dt), NULL, NULL);
    mean_h=0.0f; rtc_data_add_par("mean_h", &mean_h, RTC_TYPE_FLOAT, sizeof(mean_h), NULL, NULL);
	
	x_Kp = 0.0f; rtc_data_add_par("x_Kp", &x_Kp, RTC_TYPE_FLOAT, sizeof(x_Kp), NULL, NULL);
	x_Kd = 0.0f; rtc_data_add_par("x_Kd", &x_Kd, RTC_TYPE_FLOAT, sizeof(x_Kd), NULL, NULL);
	x1_control = 0; rtc_data_add_par("x1_control", &x1_control, RTC_TYPE_INT32, sizeof(x1_control), NULL, NULL);
	
	memset(x1_target_coeffs, 0, sizeof(x1_target_coeffs)); rtc_data_add_par("x1_target_coeffs", &x1_target_coeffs, RTC_TYPE_FLOAT, sizeof(x1_target_coeffs), NULL, NULL);
	memset(x1_filter_state, 0, sizeof(x1_filter_state));
	memset(x1d_filter_state, 0, sizeof(x1d_filter_state));
	memset(x1_target_filter_state, 0, sizeof(x1_target_filter_state));	
	memset(x1d_target_filter_state, 0, sizeof(x1d_target_filter_state));	
	memset(x1_error_filter_state, 0, sizeof(x1_error_filter_state));
	memset(x1_error_dt_filter_state, 0, sizeof(x1_error_dt_filter_state));

	memset(&x1_fourier, 0, sizeof(x1_fourier));
	memset(x1_coeffs, 0, sizeof(x1_coeffs)); rtc_data_add_par("x1_coeffs", x1_coeffs, RTC_TYPE_FLOAT, sizeof(x1_coeffs), NULL, NULL);
	memset(x1_coeffs_ave, 0, sizeof(x1_coeffs_ave)); rtc_data_add_par("x1_coeffs_ave", x1_coeffs_ave, RTC_TYPE_FLOAT, sizeof(x1_coeffs_ave), NULL, NULL);
	memset(x1_coeffs_var, 0, sizeof(x1_coeffs_var)); rtc_data_add_par("x1_coeffs_var", x1_coeffs_var, RTC_TYPE_FLOAT, sizeof(x1_coeffs_var), NULL, NULL);
	memset(x1_coeffs_arr, 0, sizeof(x1_coeffs_arr)); rtc_data_add_par("x1_coeffs_arr", x1_coeffs_arr, RTC_TYPE_FLOAT, sizeof(x1_coeffs_arr), NULL, NULL);
	x1_coeffs_idx = 0;
	
	

	/* variables relating to the output */
	out = 0.0f; rtc_data_add_par("out", &out, RTC_TYPE_FLOAT, sizeof(out), NULL, NULL);
	memset(&out_fourier, 0, sizeof(out_fourier));
	memset(out_coeffs, 0, sizeof(out_coeffs)); rtc_data_add_par("out_coeffs", &out_coeffs, RTC_TYPE_FLOAT, sizeof(out_coeffs), NULL, NULL);
	memset(out_coeffs_ave, 0, sizeof(out_coeffs_ave)); rtc_data_add_par("out_coeffs_ave", &out_coeffs_ave, RTC_TYPE_FLOAT, sizeof(out_coeffs_ave), NULL, NULL);
	memset(out_coeffs_var, 0, sizeof(out_coeffs_var)); rtc_data_add_par("out_coeffs_var", &out_coeffs_var, RTC_TYPE_FLOAT, sizeof(out_coeffs_var), NULL, NULL);
	memset(out_coeffs_arr, 0, sizeof(out_coeffs_arr)); rtc_data_add_par("out_coeffs_arr", &out_coeffs_arr, RTC_TYPE_FLOAT, sizeof(out_coeffs_arr), NULL, NULL);
	out_coeffs_idx = 0;

	/* variables relating to the random input */
	rand_ampl = 0.0f; rtc_data_add_par("rand_ampl", &rand_ampl, RTC_TYPE_FLOAT, sizeof(rand_ampl), NULL, NULL);
	rand_seed_w = 0x70123413; rtc_data_add_par("rand_seed_w", &rand_seed_w, RTC_TYPE_UINT32, sizeof(rand_seed_w), NULL, NULL);
	rand_seed_z = 0x10300013; rtc_data_add_par("rand_seed_z", &rand_seed_z, RTC_TYPE_UINT32, sizeof(rand_seed_z), NULL, NULL);
	rand_filter_freq = 0.01f; rtc_data_add_par("rand_filter_freq", &rand_filter_freq, RTC_TYPE_FLOAT, sizeof(rand_filter_freq), NULL, NULL);
	memset(&rand_filter, 0, sizeof(rand_filter));
	memset(rand_filter_state, 0, sizeof(rand_filter_state));


	/* set up the filters */
	update_filter(&input_filter, input_filter_freq);
	input_filter.order = INPUT_FILTER_ORDER;
	update_filter(&rand_filter, rand_filter_freq);
	rand_filter.order = RAND_FILTER_ORDER;

}

/* ************************************************************************ */
void rtc_user_main(void)
{
	static int led = 0;
	int i, j;
	
	
	/* ********************************************************************** */
	/*  Pre-calculations */
	/* ********************************************************************** */

	/* calculate sinusoids for use in the Fourier calculations */
	calc_sinusoids();

	/* ********************************************************************** */
	/*  Real-time control */
	/* ********************************************************************** */

	/* Fshaker */
	Fshaker = in_volt[1];
       
	/* calculate relative heave */
	x1_raw = in_volt[0];
	
	x1 = biquad_filter(x1_raw, &input_filter, x1_filter_state);

	
	x1d_raw=(x1-x1_last)*TIMER_FREQ;
	x1_last=x1;
	
	x1d = biquad_filter(x1d_raw, &input_filter, x1d_filter_state);

	 /* section in which the angle phi_bis to be used in the target is evaluated */
	 
	 /* angle in the phase plane x1d, x1 considering a zero mean for x1 */
	if ((x1-mean_h)<0) 
        phi=atan2(x1d,(x1-mean_h))+M_PI;
    else
        phi=atan2(x1d,(x1-mean_h));
    
     /* angle in the phase plane x1d, x1 to be adopted for the target given the frequency adopted in the parametrization 'forcing_freq' */
    if ((x1-mean_h)<0) 
        phi_bis=atan2(forcing_freq*M_2PI,tan(phi))+M_PI;
    else
		phi_bis=atan2(forcing_freq*M_2PI,tan(phi));
	
    time_star=1/(forcing_freq*M_2PI)*phi_bis;
	
	
    /* calculate the x1_target and derivative at the same angle phi*/
    x1_target= x1_target_coeffs[8]+x1_target_coeffs[1]*sin(phi_bis);

    x1d_target=x1_target_coeffs[1]*forcing_freq*M_2PI*cos(phi_bis);


	/* calculate x1_error_raw */
	if (x1_control)
		x1_error_raw = x1_target - x1;
	else
		x1_error_raw = 0.0f;

	/* filter to get the error for control purposes */
	x1_error = biquad_filter(x1_error_raw, &input_filter, x1_error_filter_state); 

	/* calculate derror/dt - finite difference */
	x1_error_dt_raw = (x1_error - x1_error_last)*TIMER_FREQ; /*(x1d_target-x1d);/*(x1_error - x1_error_last)*TIMER_FREQ; */
	x1_error_last = x1_error;

	/* filter to get the derivative error for control purposes */
	x1_error_dt = biquad_filter(x1_error_dt_raw, &input_filter, x1_error_dt_filter_state);

	
	/* calculate the forcing */
	forcing = 0.0f;
	for (i = 0; i < N_FOURIER_COEFF; i++)
		forcing += sinusoid_f[i]*forcing_coeffs[i];

	/* calculate the output */
	out = x_Kp*x1_error + x_Kd*x1_error_dt + forcing;

	/* output the calculated value */
	rtc_set_output(output_channel, out);
        
	/* ********************************************************************** */
	/*  Other calculations on the input/output */
	/* ********************************************************************** */

	/* calculate the Fourier coefficients of the relative motion */
	update_fourier(x1, x1_coeffs, &x1_fourier);
	
	/* calculate the Fourier coefficients of the output */
	update_fourier(out, out_coeffs, &out_fourier);



	/* update averages */
	if (period_start) {
		for (i = 0; i < N_FOURIER_COEFF; i++) {
			/* store the current data sample */
			x1_coeffs_arr[x1_coeffs_idx][i] = x1_coeffs[i];
			out_coeffs_arr[out_coeffs_idx][i] = out_coeffs[i];
			/* mean */
			x1_coeffs_ave[i] = 0.0f;
			out_coeffs_ave[i] = 0.0f;
			for (j = 0; j < N_FOURIER_AVE; j++) {
				x1_coeffs_ave[i] += x1_coeffs_arr[j][i];
				out_coeffs_ave[i] += out_coeffs_arr[j][i];
			}
			x1_coeffs_ave[i] *= (1.0f/N_FOURIER_AVE);
			out_coeffs_ave[i] *= (1.0f/N_FOURIER_AVE);
			/* variance */
			x1_coeffs_var[i] = 0.0f;
			out_coeffs_var[i] = 0.0f;
			for (j = 0; j < N_FOURIER_AVE; j++) {
				x1_coeffs_var[i] += (x1_coeffs_arr[j][i] - x1_coeffs_ave[i])*(x1_coeffs_arr[j][i] - x1_coeffs_ave[i]);
				out_coeffs_var[i] += (out_coeffs_arr[j][i] - out_coeffs_ave[i])*(out_coeffs_arr[j][i] - out_coeffs_ave[i]);
			}
			x1_coeffs_var[i] *= (1.0f/N_FOURIER_AVE);
			out_coeffs_var[i] *= (1.0f/N_FOURIER_AVE);
		}
		/* increment indices */
		x1_coeffs_idx++;
		out_coeffs_idx++;
		if (x1_coeffs_idx == N_FOURIER_AVE)
			x1_coeffs_idx = 0;
		if (out_coeffs_idx == N_FOURIER_AVE)
			out_coeffs_idx = 0;
	     mean_h=x1_coeffs_ave[N_FOURIER_MODES+1]; 
	}

	/* ********************************************************************** */
	/*  Update time */
	/* ********************************************************************** */

	period_start = 0;
	time_mod_2pi += time_delta*forcing_freq;
	if (time_mod_2pi > M_2PI) {
		time_mod_2pi -= M_2PI;
		rtc_led(2, led);
		led = !led;
		period_start = 1;
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


/* ********************************************************************** 
	Update Fourier components
   ********************************************************************** */
void update_fourier(float value_f, float output_f[N_FOURIER_COEFF], struct fourier_t *fourier)
{
	float tmp;
	int i;

	/* recursion for numerical stability (reduces discretisation x_error) */
	for (i = 0; i < N_FOURIER_COEFF; i++)
		value_f -= output_f[i]*sinusoid_f[i];
	/* multiply by sinusoids & update integral */
	for (i = 0; i < N_FOURIER_COEFF; i++) {
		tmp = value_f*sinusoid_f[i];
		fourier->integrator[i] += fourier->last_value[i] + tmp;
		fourier->last_value[i] = tmp;
	}
	/* update integral */
	if (period_start) {
		float norm_forcing_freq = forcing_freq*TIMER_PERIOD_FLOAT;
		fourier->integrator[N_FOURIER_COEFF/2] *= 0.5;  /* adjustment for the DC level */
		for (i = 0; i < N_FOURIER_COEFF; i++) {
			output_f[i] += norm_forcing_freq*fourier->integrator[i];
			fourier->integrator[i] = 0.0f;
		}
	}
}


/* ********************************************************************** 
	A biquad filter
   ********************************************************************** */
float biquad_filter(float x, const struct biquad_filter_t *filter, float state[])
{
	int i;
	float y = x;
	for (i = 0; i < filter->order; i++) {
		float w = y + filter->a[0]*state[2*i+0] + filter->a[1]*state[2*i+1];
		y = filter->b[0]*w + filter->b[1]*state[2*i+0] + filter->b[2]*state[2*i+1];
		state[2*i+1] = state[2*i+0];
		state[2*i+0] = w;
	}
	return y;
}

/* ********************************************************************** 
	Random number generation
   ********************************************************************** */
unsigned int get_random()
{
	rand_seed_z = 36969 * (rand_seed_z & 65535) + (rand_seed_z >> 16);
	rand_seed_w = 18000 * (rand_seed_w & 65535) + (rand_seed_w >> 16);
	return (rand_seed_z << 16) + rand_seed_w;  /* 32-bit result */
}

/* ********************************************************************** 
	Calculate 1/tan(pi*x)
   ********************************************************************** */
float inv_tan_pi(float x)
{
	v4sf t MEM_ALIGN, sine MEM_ALIGN, cosine MEM_ALIGN;
	float tt[4] MEM_ALIGN, sine_f[4] MEM_ALIGN, cosine_f[4] MEM_ALIGN;

	tt[0] = x; tt[1] = 0.0f; tt[2] = 0.0f; tt[3] = 0.0f;
	t = vld1q_f32(tt);
	t = vmulq_f32(t, vdupq_n_f32(M_PI));
	sincos_ps(t, &sine, &cosine);
	vst1q_f32(sine_f, sine);
	vst1q_f32(cosine_f, cosine);
	return (cosine_f[0]/sine_f[0]);
}

/* ********************************************************************** 
	Calculate the Butterworth filter coefficients for a normalised
	frequency value (0 < freq < 1)
   ********************************************************************** */
void update_filter(struct biquad_filter_t *filter, float freq)
{
	float ita, q = 1.414213562f, b0;
	/* Check that the filter cut off is between 0 and 1 */
	if ((freq < 0.0f) || (freq >= 1.0f))
		freq = 0.01f; /* default value */
    ita = inv_tan_pi(freq);
    b0 = 1.0f / (1.0f + q*ita + ita*ita);
    filter->a[0] = 2.0f*(ita*ita - 1.0f)*b0;
    filter->a[1] = -(1.0f - q*ita + ita*ita)*b0;
    filter->b[0] = b0;
    filter->b[1] = 2.0f*b0;
    filter->b[2] = b0;
}

/* ********************************************************************** 
	Update the coefficients of the input filter
   ********************************************************************** */
void update_input_filter(void *new_value, void *trigger_data)
{
	float freq = *(float *)new_value;
	if ((freq > 0.0f) && (freq <= 1.0f)) {
		input_filter_freq = freq;
		update_filter(&input_filter, freq);
	}
}

/* ********************************************************************** 
	Update the coefficients of the random number filter
   ********************************************************************** */
void update_rand_filter(void *new_value, void *trigger_data)
{
	float freq = *(float *)new_value;
	if ((freq > 0.0f) && (freq <= 1.0f)) {
		rand_filter_freq = freq;
		update_filter(&rand_filter, freq);
	}
}
