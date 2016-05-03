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
#include <stdlib.h>
#include "rtc_user.h"
#include "rtc_util.h"
#include "rtc_main.h"
#include "rtc_data.h"
#include "ad760x.h"
#include "ad5064.h"
#include "interrupt.h"

#define OUTPUT_VOLTAGE 4.096f

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "none"
#endif

static char firmware[] = FIRMWARE_VERSION;
static unsigned int rtc_sample_freq;
static volatile unsigned int rtc_clock_jitter;
static volatile unsigned int rtc_user_overrun;
static volatile unsigned int rtc_user_max_time;
static volatile unsigned int rtc_user_last_time;
static volatile unsigned int rtc_input_voltage_range;
static volatile float rtc_input_voltage_scale;
static volatile unsigned int rtc_last_time;
volatile unsigned int rtc_user_finished;
volatile float in_volt[AD760X_CHANNEL_COUNT];
volatile float out_volt[4];

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
static void rtc_set_input_voltage_range(void *new_value, void *trigger_data);

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void rtc_user_init_handler(void)
{
	char input_name[] = "I#volt";
	int i;
	ad760xDataHandler = rtc_user_main_handler;
	rtc_data_add_par("firmware", &firmware, RTC_TYPE_CHAR, sizeof(firmware), rtc_data_trigger_read_only, NULL);
	rtc_sample_freq = TIMER_FREQ;
	rtc_data_add_par("sample_freq", &rtc_sample_freq, RTC_TYPE_UINT32, sizeof(unsigned int), rtc_data_trigger_read_only, NULL);
	rtc_last_time = 0;
	rtc_data_add_par("time_raw", &rtc_last_time, RTC_TYPE_UINT32, sizeof(unsigned int), rtc_data_trigger_read_only, NULL);
	rtc_clock_jitter = 0;
	rtc_data_add_par("clock_jitter", &rtc_clock_jitter, RTC_TYPE_UINT32, sizeof(unsigned int), NULL, NULL);
	rtc_user_overrun = 0;
	rtc_data_add_par("overrun", &rtc_user_overrun, RTC_TYPE_UINT32, sizeof(unsigned int), NULL, NULL);
	rtc_user_max_time = 0;
	rtc_data_add_par("user_max_time", &rtc_user_max_time, RTC_TYPE_UINT32, sizeof(unsigned int), NULL, NULL);
	rtc_user_last_time = 0;
	rtc_data_add_par("user_last_time", &rtc_user_last_time, RTC_TYPE_UINT32, sizeof(unsigned int), NULL, NULL);
	rtc_input_voltage_range = 5;
	rtc_data_add_par("in_volt_range", &rtc_input_voltage_range, RTC_TYPE_UINT32, sizeof(unsigned int), rtc_set_input_voltage_range, NULL);
	rtc_set_input_voltage_range((void *)&rtc_input_voltage_range, NULL);
	for (i = 0; i < AD760X_CHANNEL_COUNT; i++) {
		input_name[1] = '0' + i;
		rtc_data_add_par(input_name, &in_volt[i], RTC_TYPE_FLOAT, sizeof(float), rtc_data_trigger_read_only, NULL);
	}
	input_name[0] = 'O';
	for (i = 0; i < 4; i++) {
		input_name[1] = '0' + i;
		rtc_data_add_par(input_name, &out_volt[i], RTC_TYPE_FLOAT, sizeof(float), rtc_data_trigger_read_only, NULL);
	}
	rtc_user_finished = 0;
	rtc_user_init();
}

/* ************************************************************************ */
void rtc_user_main_handler(void)
{
	static int first_call = 100;
	int i;
	unsigned int t1, t2;
	unsigned char IntStatus;

	/* Turn off interrupts temporarily */
	IntStatus = IntDisable();

	/* Acknowledge data received */
	ad760xBufferReady = 0;

	/* Check clock jitter - ignore the first n cycles for the CPU cache to kick in */
	t1 = TIME;
	if (first_call == 0) {
		t2 = t1 - rtc_last_time;
		if (t2 > rtc_clock_jitter + TIMER_PERIOD)
			rtc_clock_jitter = t2 - TIMER_PERIOD;
	}
	else
		first_call--;
	rtc_last_time = t1;

	/* Calculate the actual voltages */
	for (i = 0; i < AD760X_CHANNEL_COUNT; i++)
		in_volt[i] = ((float)ad760xBuffer[i])*rtc_input_voltage_scale;

	/* Call the user code */
	rtc_user_main();

	/* See how long we took */
	t2 = TIME;

	/* Store timings */
	if (first_call == 0) {
		rtc_user_last_time = t2 - t1;
		if (rtc_user_last_time > rtc_user_max_time)
			rtc_user_max_time = rtc_user_last_time;
		if (rtc_user_last_time > (TIMER_PERIOD/2)) {
			usb_print("Possible overrun\r\n");
			rtc_user_overrun++;
		}
	}

	/* Update stream capture variables */
	rtc_data_update_streams();

	/* Increment the time counter */
	time_ctr += 1;

	/* Finished */
	rtc_user_finished = 1;

	/* Re-enable interrupts */
	IntEnable(IntStatus);
}

/* ************************************************************************ */
void rtc_set_input_voltage_range(void *new_value, void *trigger_data)
{
	unsigned int voltage = *(unsigned int *)new_value;
	if (voltage == 10) {
		ad760xSetRange(AD760X_RANGE_HIGH);
		rtc_input_voltage_range = 10;
		rtc_input_voltage_scale = ((float)(AD760X_FSR_HIGH))/(1 << AD760X_BIT_COUNT);
	}
	else {
		ad760xSetRange(AD760X_RANGE_LOW);
		rtc_input_voltage_range = 5;
		rtc_input_voltage_scale = ((float)(AD760X_FSR_LOW))/(1 << AD760X_BIT_COUNT);
	}
}

/* ************************************************************************ */
void rtc_set_output(unsigned int channel, float value)
{
	/* Set the analogue output of the board */
	if (channel >= 4)
		return;
	switch (channel) {
		/* Channels 1 & 3 are bipolar outputs */
		case 0:
		case 2:
			if (value < -OUTPUT_VOLTAGE)
				value = -OUTPUT_VOLTAGE;
			else if (value > OUTPUT_VOLTAGE)
				value = OUTPUT_VOLTAGE;
			out_volt[channel] = value;
			ad5064SetOutput(channel, (unsigned int)((value + OUTPUT_VOLTAGE)*(((float)0xFFFF)/(2*OUTPUT_VOLTAGE))));
			break;
		/* Channels 2 & 4 are unipolar outputs */
		case 1:
		case 3:
			if (value < 0)
				value = 0;
			else if (value > OUTPUT_VOLTAGE)
				value = OUTPUT_VOLTAGE;
			out_volt[channel] = value;
			ad5064SetOutput(channel, (unsigned int)(value*(((float)0xFFFF)/OUTPUT_VOLTAGE)));
			break;
	}
}
