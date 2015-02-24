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
#include "rtc_data.h"
#include "rtc_messaging.h"
#include "rtc_main.h"
#include "rtc_user.h"
#include "rtc_util.h"
#include "ad760x.h"
#include "ad5064.h"
#include <string.h>


#define MAX_PARS 256  /* maximum number of parameters */
#define MAX_PAR_NAMES (16*MAX_PARS)  /* space allocated for parameter names */

#define MAX_STREAMS 4  /* Assumed to be < 10 for ease of implementation */
#define MAX_STREAM_CHANNELS 10
#define MAX_STREAM_BUFFER (8*1024*1024)

static unsigned int     rtc_par_count;
static char             rtc_par_names[MAX_PAR_NAMES];
static unsigned int     rtc_par_names_idx;
static volatile void    *rtc_par_ptr[MAX_PARS];
static unsigned char    rtc_par_type[MAX_PARS];
static unsigned int     rtc_par_size[MAX_PARS];
static rtc_trigger_func *rtc_par_trigger_func[MAX_PARS];
static void             *rtc_par_trigger_data[MAX_PARS];

#define RTC_STREAM_STATE_INACTIVE 0
#define RTC_STREAM_STATE_ACTIVE   1
#define RTC_STREAM_STATE_FINISHED 2

typedef struct {
	unsigned int state;
	unsigned int next_time;
	unsigned int downsample;
	unsigned int samples;
	unsigned int samples_remaining;
	unsigned int nparams;
	unsigned int params[MAX_STREAM_CHANNELS];
	unsigned int data[MAX_STREAM_BUFFER];
	unsigned int *data_ptr;
	unsigned int data_size;
} rtc_stream;

static rtc_stream rtc_streams[MAX_STREAMS];

/* ************************************************************************ */
/* * Internal prototypes ************************************************** */
/* ************************************************************************ */
static void rtc_stream_change_state(void *new_value, void *trigger_data);

/* ************************************************************************ */
/* * Function definitions ************************************************* */
/* ************************************************************************ */
void rtc_data_init(void)
{
	int i, j;
	char state_name[] = "S#state";
	char samples_name[] = "S#samples";
	char downsample_name[] = "S#downsample";
	char params_name[] = "S#params";
	char channel_name[] = "I#raw";

	rtc_par_count = 0;
	rtc_par_names_idx = 0;
	
	/* Add the master time counter as a parameter */
	rtc_data_add_par("time_ctr", &time_ctr, RTC_TYPE_UINT32, sizeof(unsigned int), rtc_data_trigger_read_only, NULL);

	/* Add the input channels */
	for (i = 0; i < AD760X_CHANNEL_COUNT; i++) {
		channel_name[1] = '0' + i;
		rtc_data_add_par(channel_name, &ad760xBuffer[i], RTC_TYPE_INT32, sizeof(int), rtc_data_trigger_read_only, NULL);
	}

	/* Add the output channels */
	channel_name[0] = 'O';
	for (i = 0; i < 4; i++) {
		channel_name[1] = '0' + i;
		rtc_data_add_par(channel_name, &ad5064Buffer[i], RTC_TYPE_UINT32, sizeof(unsigned int), NULL, NULL);
	}

	/* Initialise the streams */
	for (i = 0; i < MAX_STREAMS; i++) {
		/* Zero any host-visible streams (except data) */
		rtc_streams[i].state = RTC_STREAM_STATE_INACTIVE;
		rtc_streams[i].samples = 0;
		rtc_streams[i].downsample = 0;
		rtc_streams[i].nparams = 0;
		for (j = 0; j < MAX_STREAM_CHANNELS; j++)
			rtc_streams[i].params[j] = 0;
		/* The names of the parameters */
		state_name[1] = '0' + i;
		samples_name[1] = '0' + i;
		downsample_name[1] = '0' + i;
		params_name[1] = '0' + i;
		/* Add the relevant fields as parameters */
		rtc_data_add_par(state_name, &rtc_streams[i].state, RTC_TYPE_UINT32, sizeof(unsigned int), rtc_stream_change_state, (void *)i);
		rtc_data_add_par(samples_name, &rtc_streams[i].samples, RTC_TYPE_UINT32, sizeof(unsigned int), NULL, NULL);
		rtc_data_add_par(downsample_name, &rtc_streams[i].downsample, RTC_TYPE_UINT32, sizeof(unsigned int), NULL, NULL);
		rtc_data_add_par(params_name, rtc_streams[i].params, RTC_TYPE_UINT32, sizeof(unsigned int)*MAX_STREAM_CHANNELS, NULL, NULL);
	}
}

/* ************************************************************************ */
unsigned int rtc_data_add_par(const char *name, volatile void *ptr, unsigned char type, 
		unsigned int size, rtc_trigger_func *trigger_func, void *trigger_data)
{
	int name_len = strlen(name);
	/* Check that there aren't too many parameters allocated */
	if (rtc_par_count >= MAX_PARS)
		return 0;
	/* Check that there is sufficient space in the name buffer */
	if ((rtc_par_names_idx + name_len) >= MAX_PAR_NAMES)
		return 0;
	/* Copy the data including a null terminator */
	strncpy(&rtc_par_names[rtc_par_names_idx], name, name_len + 1);
	rtc_par_names_idx += name_len + 1;
	/* Store the other attributes */
	rtc_par_ptr[rtc_par_count] = ptr;
	rtc_par_type[rtc_par_count] = type;
	rtc_par_size[rtc_par_count] = size;
	rtc_par_trigger_func[rtc_par_count] = trigger_func;
	rtc_par_trigger_data[rtc_par_count] = trigger_data;
	/* Update parameter count */
	rtc_par_count += 1;
	return rtc_par_count - 1;
}

/* ************************************************************************ */
void rtc_data_trigger_read_only(void *new_value, void *trigger_data)
{
	/* Do nothing; ignore the write request */
	(void)new_value;
	(void)trigger_data;
}

/* ************************************************************************ */
void rtc_data_update_streams(void)
{
	unsigned int i, j;
	/* Iterate through the streams and update any that are active */
	for (i = 0; i < MAX_STREAMS; i++) {
		if (rtc_streams[i].state == RTC_STREAM_STATE_ACTIVE) {
			if (time_ctr >= rtc_streams[i].next_time) {
				/* If the next sample time has occurred, store a sample */
				for (j = 0; j < rtc_streams[i].nparams; j++)
					*rtc_streams[i].data_ptr++ = *((unsigned int *)rtc_par_ptr[rtc_streams[i].params[j]]);
				rtc_streams[i].next_time = time_ctr + rtc_streams[i].downsample + 1;
				rtc_streams[i].samples_remaining--;
			}
			if (rtc_streams[i].samples_remaining == 0)
				/* If the end time has occurred, set the stream to inactive */
				rtc_streams[i].state = RTC_STREAM_STATE_FINISHED;
		}
	}
}

/* ************************************************************************ */
void rtc_stream_change_state(void *new_value, void *trigger_data)
{
	/* Extract the data from the pointers */
	unsigned int new_state = *((unsigned int *)new_value);
	unsigned int idx = (unsigned int)trigger_data;
	int i;
	/* Work out what to do */
	switch (new_state) {
		case RTC_STREAM_STATE_ACTIVE:
			/* Start data capture */
			rtc_streams[idx].samples_remaining = rtc_streams[idx].samples;
			rtc_streams[idx].next_time = time_ctr;
			for (i = 0; i < MAX_STREAM_CHANNELS; i++) {
				if (rtc_streams[idx].params[i] >= rtc_par_count)
					break;
				if (rtc_par_size[rtc_streams[idx].params[i]] != 4)
					return;
			}
			rtc_streams[idx].nparams = i;
			rtc_streams[idx].data_ptr = rtc_streams[idx].data;
			/* Calculate how much data will be captured */
			rtc_streams[idx].data_size = 4*rtc_streams[idx].nparams*rtc_streams[idx].samples;
			/* Check that we will capture some data but not too much */
			if ((rtc_streams[idx].nparams > 0) && (rtc_streams[idx].data_size <= MAX_STREAM_BUFFER) && (rtc_streams[idx].samples != 0))
				rtc_streams[idx].state = RTC_STREAM_STATE_ACTIVE;
			break;
		case RTC_STREAM_STATE_FINISHED:
		case RTC_STREAM_STATE_INACTIVE:
			rtc_streams[idx].state = RTC_STREAM_STATE_INACTIVE;
			break;
	}
}

/* ************************************************************************ */
int rtc_data_get_par_names()
{
	tx_buffer_ptr = rtc_par_names;
	tx_size = rtc_par_names_idx;
	return 0;
}

/* ************************************************************************ */
int rtc_data_get_par_types()
{
	tx_buffer_ptr = rtc_par_type;
	tx_size = sizeof(unsigned char)*rtc_par_count;
	return 0;
}

/* ************************************************************************ */
int rtc_data_get_par_sizes()
{
	tx_buffer_ptr = rtc_par_size;
	tx_size = sizeof(unsigned int)*rtc_par_count;
	return 0;
}

/* ************************************************************************ */
int rtc_data_get_par_value()
{
	unsigned int par_id;
	unsigned char *rx_ptr = rx_buffer;
	unsigned char *tx_ptr = tx_buffer;
	/* Try to not get interrupted by running immediately after the user code has finished */
	rtc_user_finished = 0;
	while (!rtc_user_finished);
	/* Get the requested data values */
	while (rx_size >= 4) {
		par_id = *(unsigned int *)rx_ptr;
		rx_ptr += 4;
		rx_size -= 4;
		if (par_id >= rtc_par_count)
			return -1;
		/* Check size constraints on the tx buffer */
		tx_size += rtc_par_size[par_id];
		if (tx_size > MAX_TX_BUFFER)
			return -1;
		/* Return the data in the tx buffer */
		if (rtc_par_size[par_id] == 4)
			/* Common use case */
			*(unsigned int *)tx_ptr = *(unsigned int *)rtc_par_ptr[par_id];
		else
			/* Arbitrary sizes */
			memcpy(tx_ptr, (void *)rtc_par_ptr[par_id], rtc_par_size[par_id]);
		tx_ptr += rtc_par_size[par_id];
	}
	tx_buffer_ptr = tx_buffer;
	return 0;
}

/* ************************************************************************ */
int rtc_data_set_par_value()
{
	unsigned char *rx_ptr = rx_buffer;
	unsigned int par_id;
	/* Try to not get interrupted by running immediately after the user code has finished */
	rtc_user_finished = 0;
	while (!rtc_user_finished);
	/* Set the requested data values */	
	while (rx_size >= 4) {
		par_id = *(unsigned int *)rx_ptr;
		rx_ptr += 4;
		rx_size -= 4;
		if (par_id >= rtc_par_count)
			return -1;
		/* Check that there is enough data in the rx buffer */
		if (rx_size < rtc_par_size[par_id])
			return -1;
		rx_size -= rtc_par_size[par_id];
		/* If there is a trigger associated with this function, call it */
		if (rtc_par_trigger_func[par_id] != NULL)
			rtc_par_trigger_func[par_id](rx_ptr, rtc_par_trigger_data[par_id]);
		else {
			/* Put the data into the variable */
			if (rtc_par_size[par_id] == 4)
				/* Common use case */
				*(unsigned int *)rtc_par_ptr[par_id] = *(unsigned int *)rx_ptr;
			else
				/* Arbitrary sizes */
				memcpy((void *)rtc_par_ptr[par_id], rx_ptr, rtc_par_size[par_id]);
		}
		rx_ptr += rtc_par_size[par_id];
	}
	if (rx_size != 0)
		return -1;
	else
		return 0;
}

/* ************************************************************************ */
int rtc_data_get_stream()
{
	unsigned int stream_id;
	if (rx_size != 4)
		return -1;
	else {
		void *rx_buffer_ptr = rx_buffer;
		stream_id = *(unsigned int *)rx_buffer_ptr;
		tx_buffer_ptr = rtc_streams[stream_id].data;
		tx_size = rtc_streams[stream_id].data_size;
		return 0;
	}
}
