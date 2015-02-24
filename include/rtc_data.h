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
#ifndef __RTC_DATA_H__
#define __RTC_DATA_H__

/* ************************************************************************ */
/* * Includes, defines and global variables ******************************* */
/* ************************************************************************ */
#define RTC_TYPE_INT8   'b'
#define RTC_TYPE_INT16  'h'
#define RTC_TYPE_INT32  'i'
#define RTC_TYPE_UINT8  'B'
#define RTC_TYPE_UINT16 'H'
#define RTC_TYPE_UINT32 'I'
#define RTC_TYPE_CHAR   'c'
#define RTC_TYPE_FLOAT  'f'


/* ************************************************************************ */
/* * Prototypes *********************************************************** */
/* ************************************************************************ */

/* 
  Signature of the trigger function; first parameter is a pointer to the new
  value sent by the host, second parameter is trigger_data provided to 
  rtc_data_add_par.
*/
typedef void (rtc_trigger_func)(void *, void *);

/* Initialise the real-time data system */
void rtc_data_init(void);

/* 
   Add a parameter to the list of accessible real-time parameters; name should
   be null terminated; trigger_func is a function to be called if the value is
   changed by the host. If a trigger function is provided, the new value 
   *will not* be automatically transferred into the variable.
*/
unsigned int rtc_data_add_par(const char *name, volatile void *ptr, unsigned char type, 
		unsigned int size, rtc_trigger_func *trigger_func, void *trigger_data);

/* Put the parameter names into the messaging send buffer */
int rtc_data_get_par_names();

/* Put the parameter types into the messaging send buffer */
int rtc_data_get_par_types();

/* Put the parameter sizes into the messaging send buffer */
int rtc_data_get_par_sizes();

/* Put the parameter value requested into the messaging send buffer */
int rtc_data_get_par_value();

/* Set the parameter value requested with the contents of the receive buffer */
int rtc_data_set_par_value();

/* Put the stream requested into the messaging send buffer */
int rtc_data_get_stream();

/* A trigger function for use with read only parameters */
void rtc_data_trigger_read_only(void *new_value, void *trigger_data);

/* Capture data (if necessary) - expects to be called after msg_poll */
void rtc_data_update_streams(void);


#endif
