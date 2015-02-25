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

/* Variables relating to the input filter */
static float input_filter_freq;
static struct biquad_filter_t input_filter;

/* Variables relating to the relative mass position (x) */
static float x;  /* x motion */
static float x_target;  /* Target for x (set by the Fourier coefficients) */
static float x_error_filter_state[INPUT_FILTER_N_STATE];  /* Filtering for the error in x motion */
static float x_error_raw, x_error, x_error_last, x_error_dt;  /* Control error */
static float x_target_coeffs[N_FOURIER_COEFF] MEM_ALIGN;  /* Fourier coeffs of the x_target */
static float x_Kp;  /* Proportional control gain */
static float x_Kd;  /* Derivative control gain */
static int x_control;  /* Whether control is switched on or off */

static struct fourier_t x_fourier;  /* Internal structure for Fourier estimation */
static float x_coeffs[N_FOURIER_COEFF] MEM_ALIGN;  /* Fourier coeffs of x */
static float x_coeffs_ave[N_FOURIER_COEFF] MEM_ALIGN;  /* Average Fourier coeffs of x */
static float x_coeffs_var[N_FOURIER_COEFF] MEM_ALIGN;  /* Variance of Fourier coeffs of x */
static float x_coeffs_arr[N_FOURIER_AVE][N_FOURIER_COEFF] MEM_ALIGN;  /* Last N Fourier coeffs of x */
static int x_coeffs_idx;

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

/* Variables relating to the simulated system */
static float z, dz;
static float alpha, beta, gamma;


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
	memset(&input_filter, 0, sizeof(input_filter));

	/* variables relating to the relative mass position (x) */
	x = 0.0f; rtc_data_add_par("x", &x, RTC_TYPE_FLOAT, sizeof(x), NULL, NULL);
	x_target = 0.0f; rtc_data_add_par("x_target", &x_target, RTC_TYPE_FLOAT, sizeof(x_target), NULL, NULL);
	x_error_raw = 0.0f; rtc_data_add_par("x_error_raw", &x_error_raw, RTC_TYPE_FLOAT, sizeof(x_error_raw), NULL, NULL);
	x_error = 0.0f; rtc_data_add_par("x_error", &x_error, RTC_TYPE_FLOAT, sizeof(x_error), NULL, NULL);
	x_error_last = 0.0f;
	x_error_dt = 0.0f; rtc_data_add_par("x_error_dt", &x_error_dt, RTC_TYPE_FLOAT, sizeof(x_error_dt), NULL, NULL);
	x_Kp = 0.0f; rtc_data_add_par("x_Kp", &x_Kp, RTC_TYPE_FLOAT, sizeof(x_Kp), NULL, NULL);
	x_Kd = 0.0f; rtc_data_add_par("x_Kd", &x_Kd, RTC_TYPE_FLOAT, sizeof(x_Kd), NULL, NULL);
	x_control = 0; rtc_data_add_par("x_control", &x_control, RTC_TYPE_INT32, sizeof(x_control), NULL, NULL);
	memset(x_target_coeffs, 0, sizeof(x_target_coeffs)); rtc_data_add_par("x_target_coeffs", &x_target_coeffs, RTC_TYPE_FLOAT, sizeof(x_target_coeffs), NULL, NULL);
	memset(x_error_filter_state, 0, sizeof(x_error_filter_state));

	memset(&x_fourier, 0, sizeof(x_fourier));
	memset(x_coeffs, 0, sizeof(x_coeffs)); rtc_data_add_par("x_coeffs", x_coeffs, RTC_TYPE_FLOAT, sizeof(x_coeffs), NULL, NULL);
	memset(x_coeffs_ave, 0, sizeof(x_coeffs_ave)); rtc_data_add_par("x_coeffs_ave", x_coeffs_ave, RTC_TYPE_FLOAT, sizeof(x_coeffs_ave), NULL, NULL);
	memset(x_coeffs_var, 0, sizeof(x_coeffs_var)); rtc_data_add_par("x_coeffs_var", x_coeffs_var, RTC_TYPE_FLOAT, sizeof(x_coeffs_var), NULL, NULL);
	memset(x_coeffs_arr, 0, sizeof(x_coeffs_arr)); rtc_data_add_par("x_coeffs_arr", x_coeffs_arr, RTC_TYPE_FLOAT, sizeof(x_coeffs_arr), NULL, NULL);
	x_coeffs_idx = 0;

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

	/* variables relating to the simulated system - resonant frequency around 80 Hz */
	z = 0.0f; rtc_data_add_par("z", &z, RTC_TYPE_FLOAT, sizeof(z), NULL, NULL);
	dz = 0.0f; rtc_data_add_par("dz", &dz, RTC_TYPE_FLOAT, sizeof(dz), NULL, NULL);
	alpha = 0.05f; rtc_data_add_par("alpha", &alpha, RTC_TYPE_FLOAT, sizeof(alpha), NULL, NULL);
	beta = 1.0f; rtc_data_add_par("beta", &beta, RTC_TYPE_FLOAT, sizeof(beta), NULL, NULL);
	gamma = 1.0f; rtc_data_add_par("gamma", &gamma, RTC_TYPE_FLOAT, sizeof(gamma), NULL, NULL);
}

/* ************************************************************************ */
void rtc_user_main(void)
{
	static int led = 0;
	int i, j;
        float new_z, new_dz;
        float h = 0.0001*70*M_2PI;

	/* ********************************************************************** */
	/*  Pre-calculations */
	/* ********************************************************************** */

	/* calculate sinusoids for use in the Fourier calculations */
	calc_sinusoids();

	/* ********************************************************************** */
	/*  Real-time control */
	/* ********************************************************************** */

	/* calculate relative displacement */
	x = z;

	/* calculate the x_target (relative motion) */
	x_target = 0.0f;
	for (i = 0; i < N_FOURIER_COEFF; i++)
		x_target += sinusoid_f[i]*x_target_coeffs[i];

	/* add random noise */
	x_target += biquad_filter(rand_ampl*((float)get_random()/(float)0x7FFFFFFF - 1.0f), &rand_filter, rand_filter_state);

	/* calculate x_error_raw */
	if (x_control)
		x_error_raw = x_target - x;
	else
		x_error_raw = 0.0f;

	/* filter to get the error for control purposes */
	x_error = biquad_filter(x_error_raw, &input_filter, x_error_filter_state);

	/* calculate derror/dt - finite difference */
	x_error_dt = (x_error - x_error_last)*TIMER_FREQ;
	x_error_last = x_error;

	/* calculate the forcing */
	forcing = 0.0f;
	for (i = 0; i < N_FOURIER_COEFF; i++)
		forcing += sinusoid_f[i]*forcing_coeffs[i];

	/* calculate the output */
	out = x_Kp*x_error + x_Kd*x_error_dt + forcing;

	/* output the calculated value - numerically simulate the Duffing equation $x'' + alpha*x' + beta*x + gamma*x^3 = out$ */
        /* this uses backward Euler for the linear part and forward Euler for the nonlinear part; more stable than plain Euler */
        new_z = (-gamma*h*h*z*z*z+out*h*h+alpha*h*z+dz*h+z)/(beta*h*h+alpha*h+1);
        new_dz = (-gamma*h*z*z*z+out*h-beta*h*z+dz)/(beta*h*h+alpha*h+1);
        z = new_z;
        dz = new_dz;
        
	/* ********************************************************************** */
	/*  Other calculations on the input/output */
	/* ********************************************************************** */

	/* calculate the Fourier coefficients of the relative motion */
	update_fourier(x, x_coeffs, &x_fourier);

	/* calculate the Fourier coefficients of the output */
	update_fourier(out, out_coeffs, &out_fourier);

	/* update averages */
	if (period_start) {
		for (i = 0; i < N_FOURIER_COEFF; i++) {
			/* store the current data sample */
			x_coeffs_arr[x_coeffs_idx][i] = x_coeffs[i];
			out_coeffs_arr[out_coeffs_idx][i] = out_coeffs[i];
			/* mean */
			x_coeffs_ave[i] = 0.0f;
			out_coeffs_ave[i] = 0.0f;
			for (j = 0; j < N_FOURIER_AVE; j++) {
				x_coeffs_ave[i] += x_coeffs_arr[j][i];
				out_coeffs_ave[i] += out_coeffs_arr[j][i];
			}
			x_coeffs_ave[i] *= (1.0f/N_FOURIER_AVE);
			out_coeffs_ave[i] *= (1.0f/N_FOURIER_AVE);
			/* variance */
			x_coeffs_var[i] = 0.0f;
			out_coeffs_var[i] = 0.0f;
			for (j = 0; j < N_FOURIER_AVE; j++) {
				x_coeffs_var[i] += (x_coeffs_arr[j][i] - x_coeffs_ave[i])*(x_coeffs_arr[j][i] - x_coeffs_ave[i]);
				out_coeffs_var[i] += (out_coeffs_arr[j][i] - out_coeffs_ave[i])*(out_coeffs_arr[j][i] - out_coeffs_ave[i]);
			}
			x_coeffs_var[i] *= (1.0f/N_FOURIER_AVE);
			out_coeffs_var[i] *= (1.0f/N_FOURIER_AVE);
		}
		/* increment indices */
		x_coeffs_idx++;
		out_coeffs_idx++;
		if (x_coeffs_idx == N_FOURIER_AVE)
			x_coeffs_idx = 0;
		if (out_coeffs_idx == N_FOURIER_AVE)
			out_coeffs_idx = 0;
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
